"""Write the STEP fixtures beside this script with Open CASCADE (cadquery-ocp 7.9.3):
python3 make.py.  OCCT is a source of fixtures here and nothing of it enters the kernel."""
import os
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Transform
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SHELL, TopAbs_FACE
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_Trsf, gp_Vec
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs, STEPControl_ShellBasedSurfaceModel
from OCP.Interface import Interface_Static
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location

from OCP.STEPControl import STEPControl_Controller
STEPControl_Controller.Init_s()
HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMAS = {"ap203": "AP203", "ap214": "AP214IS", "ap242": "AP242DIS"}

def drilled():
    block = BRepPrimAPI_MakeBox(40., 30., 20.).Shape()
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(20., 15., -1.), gp_Dir(0, 0, 1)), 6., 22.).Shape()
    return BRepAlgoAPI_Cut(block, hole).Shape()

def write(shape, name, schema="AP214IS", unit="MM", mode=STEPControl_AsIs):
    Interface_Static.SetCVal_s("write.step.schema", schema)
    Interface_Static.SetCVal_s("write.step.unit", unit)
    Interface_Static.SetCVal_s("write.step.product.name", name)
    w = STEPControl_Writer()
    assert w.Transfer(shape, mode) == IFSelect_RetDone
    assert w.Write(os.path.join(HERE, name + ".step")) == IFSelect_RetDone

for key, schema in SCHEMAS.items():
    write(drilled(), "drilled_" + key, schema)
write(drilled(), "drilled_inch", "AP214IS", "INCH")
write(drilled(), "drilled_metre", "AP203", "M")
write(BRepPrimAPI_MakeBox(10., 10., 10.).Shape(), "box_ap242_foot", "AP242DIS", "FT")

# a solid with a void: a box with a sphere's shell inside it, reversed
outer = TopoDS.Shell_s(TopExp_Explorer(BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, -10), 20., 20., 20.).Shape(), TopAbs_SHELL).Current())
inner = TopoDS.Shell_s(TopExp_Explorer(BRepPrimAPI_MakeSphere(4.).Shape(), TopAbs_SHELL).Current())
mk = BRepBuilderAPI_MakeSolid(outer)
mk.Add(TopoDS.Shell_s(inner.Reversed()))
write(mk.Solid(), "void_ap214")

# a shell, written as a shell-based surface model
write(BRepPrimAPI_MakeBox(10., 10., 10.).Shape(), "shell_ap214", mode=STEPControl_ShellBasedSurfaceModel)

# an assembly of two placed parts, through XCAF (product structure)
Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
Interface_Static.SetCVal_s("write.step.unit", "MM")
doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
asm = tool.NewShape()
TDataStd_Name.Set_s(asm, TCollection_ExtendedString("pair"))
part = tool.AddShape(BRepPrimAPI_MakeBox(10., 10., 10.).Shape(), False)
TDataStd_Name.Set_s(part, TCollection_ExtendedString("cube"))
for x in (0., 25.):
    t = gp_Trsf(); t.SetTranslation(gp_Vec(x, 0., 0.))
    tool.AddComponent(asm, part, TopLoc_Location(t))
tool.UpdateAssemblies()
w = STEPCAFControl_Writer()
assert w.Transfer(doc, STEPControl_AsIs)
assert w.Write(os.path.join(HERE, "assembly_ap214.step")) == IFSelect_RetDone

# one named part through XCAF: its name is the product's
doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
part = tool.AddShape(drilled(), False)
TDataStd_Name.Set_s(part, TCollection_ExtendedString("bracket"))
w = STEPCAFControl_Writer()
assert w.Transfer(doc, STEPControl_AsIs)
assert w.Write(os.path.join(HERE, "named_ap214.step")) == IFSelect_RetDone

# a placed single part through XCAF
doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
t = gp_Trsf(); t.SetTranslation(gp_Vec(5., 6., 7.))
shape = BRepPrimAPI_MakeBox(10., 10., 10.).Shape().Moved(TopLoc_Location(t))
part = tool.AddShape(shape, False)
TDataStd_Name.Set_s(part, TCollection_ExtendedString("moved"))
w = STEPCAFControl_Writer()
assert w.Transfer(doc, STEPControl_AsIs)
assert w.Write(os.path.join(HERE, "placed_ap214.step")) == IFSelect_RetDone
