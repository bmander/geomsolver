"""Native host for the core's solved CAD recipe; invoked by solventc over stdin."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile

from OCP.Bnd import Bnd_Box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire,
                               BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepLib import BRepLib
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax1, gp_Ax2, gp_Circ, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


def checked(builder, what):
    if not builder.IsDone():
        raise ValueError(f"{what} failed in OCCT")
    return builder.Shape()


def volume(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def validate(shape, name):
    if shape.IsNull() or not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError(f"{name}: native solid is invalid")
    if shape.ShapeType() == TopAbs_SOLID:
        shape = TopoDS.Solid_s(shape)
        if not BRepLib.OrientClosedSolid_s(shape):
            raise ValueError(f"{name}: solid is open")
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    count = 0
    while explorer.More():
        solid = TopoDS.Solid_s(explorer.Current())
        if volume(solid) <= 0:
            raise ValueError(f"{name}: solid is open or has no positive volume")
        count += 1
        explorer.Next()
    if not count:
        raise ValueError(f"{name}: operation produced no solid")
    return shape


def boolean(a, b, operation):
    return checked(operation(a, b), "Boolean operation")


def face(edges):
    wire = BRepBuilderAPI_MakeWire()
    for edge in edges:
        if edge["kind"] == "line":
            builder = BRepBuilderAPI_MakeEdge(gp_Pnt(*edge["start"]), gp_Pnt(*edge["end"]))
        elif edge["kind"] == "circle":
            circle = gp_Circ(gp_Ax2(gp_Pnt(*edge["center"]), gp_Dir(*edge["normal"]),
                                    gp_Dir(*edge["x_dir"])), edge["radius"])
            builder = BRepBuilderAPI_MakeEdge(circle, *edge.get("angles", []))
        else:
            raise ValueError("unsupported CAD profile edge")
        checked(builder, "profile edge")
        wire.Add(builder.Edge())
        checked(wire, "profile wire")
    result = BRepBuilderAPI_MakeFace(wire.Wire(), True)
    checked(result, "planar profile")
    return result.Face()


def translated(shape, delta):
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(*delta))
    return checked(BRepBuilderAPI_Transform(shape, transform, True), "profile placement")


def primitive(node, shapes):
    profile = node["profile"]
    if node["kind"] == "through":
        box = Bnd_Box()
        for source in node["sources"]:
            BRepBndLib.Add_s(shapes[source], box, False)
        if box.IsVoid() or box.IsOpen():
            raise ValueError("through cutter has no finite material extent")
        bounds = box.Get()
        normal, origin = profile["normal"], profile["origin"]
        intervals = [[normal[k]*(bounds[k+j]-origin[k]) for j in (0, 3)] for k in range(3)]
        pad = math.dist(bounds[:3], bounds[3:])*4e-5
        start = sum(min(pair) for pair in intervals)-pad
        end = sum(max(pair) for pair in intervals)+pad
    elif node["kind"] == "prism":
        start, end = node["from"], node["to"]
    elif node["kind"] != "revolve":
        raise ValueError("unsupported CAD primitive")
    solids = []
    for loop in profile["loops"]:
        section = face(loop)
        if node["kind"] == "revolve":
            direction = [math.copysign(1, node["angle"])*x for x in node["axis"]]
            axis = gp_Ax1(gp_Pnt(*node["origin"]), gp_Dir(*direction))
            builder = BRepPrimAPI_MakeRevol(section, axis, abs(node["angle"]), True)
        else:
            section = translated(section, [start*x for x in profile["normal"]])
            builder = BRepPrimAPI_MakePrism(section,
                gp_Vec(*[(end-start)*x for x in profile["normal"]]), True)
        solids.append(validate(checked(builder, "profile sweep"), node["name"]))
    result = solids[0]
    for hole in solids[1:]:
        result = boolean(result, hole, BRepAlgoAPI_Cut)
    return result


def construct(recipe):
    if recipe["schema"] != 1 or recipe["units"] != "mm":
        raise ValueError("unsupported CAD recipe schema or units")
    shapes = {}
    for node in recipe["nodes"]:
        if node["id"] in shapes:
            raise ValueError("duplicate CAD node")
        try:
            if node["kind"] == "body":
                result = shapes[node["stock"]]
                for operand in node["on"]:
                    result = boolean(result, shapes[operand], BRepAlgoAPI_Fuse)
                for operand in node["cut"]:
                    result = boolean(result, shapes[operand], BRepAlgoAPI_Cut)
            else:
                result = primitive(node, shapes)
            shapes[node["id"]] = validate(result, node["name"])
        except Exception as error:
            raise ValueError(f"{node['name']}: {error}") from error
    return shapes[recipe["root"]]


def export_step(shape, output):
    # Keep an existing export intact if construction, writing or reimport fails.
    output = Path(output)
    with tempfile.TemporaryDirectory(prefix=".solvent-step-", dir=output.parent) as directory:
        temporary = Path(directory)/"solid.step"
        writer = STEPControl_Writer()
        if writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone:
            raise ValueError("STEP transfer failed")
        if writer.Write(str(temporary)) != IFSelect_RetDone:
            raise ValueError("STEP write failed")
        reader = STEPControl_Reader()
        if reader.ReadFile(str(temporary)) != IFSelect_RetDone or not reader.TransferRoots():
            raise ValueError("STEP reimport failed")
        imported = validate(reader.OneShape(), "reimported STEP")
        if not math.isclose(volume(shape), volume(imported), rel_tol=1e-7, abs_tol=1e-9):
            raise ValueError("STEP round trip changed the solid volume")
        os.replace(temporary, output)


if __name__ == "__main__":
    try:
        export_step(construct(json.load(sys.stdin)), sys.argv[1])
    except Exception as error:
        print(f"solventc CAD backend: {error}", file=sys.stderr)
        sys.exit(1)
