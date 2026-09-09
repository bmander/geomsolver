// Native solid construction and export.
#include "occt.hpp"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <StlAPI_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static gp_Pnt point(const double* p) { return gp_Pnt(p[0],p[1],p[2]); }
static gp_Dir direction(const double* p) { return gp_Dir(p[0],p[1],p[2]); }
static gp_Vec vector(const double* p) { return gp_Vec(p[0],p[1],p[2]); }
static double volume(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape,props);
    return props.Mass();
}
void validate(TopoDS_Shape& shape) {
    if (shape.IsNull() || !BRepCheck_Analyzer(shape).IsValid())
        throw std::runtime_error("native solid is invalid");
    if (shape.ShapeType() == TopAbs_SOLID) {
        auto solid = TopoDS::Solid(shape);
        if (!BRepLib::OrientClosedSolid(solid)) throw std::runtime_error("solid is open");
        shape = solid;
    }
    int count = 0;
    for (TopExp_Explorer it(shape,TopAbs_SOLID); it.More(); it.Next()) {
        double v = volume(it.Current());
        if (!std::isfinite(v) || v <= 0) throw std::runtime_error("solid has no positive volume");
        ++count;
    }
    if (!count) throw std::runtime_error("operation produced no solid");
}

extern "C" {
Cad* solvent_cad_new() noexcept { try { return new Cad; } catch (...) { return nullptr; } }
void solvent_cad_free(Cad* cad) noexcept { delete cad; }
const char* solvent_cad_error(Cad* cad) noexcept { return cad->error.c_str(); }

int solvent_cad_line(Cad* cad,const double* a,const double* b) noexcept {
    return guarded(cad,[&] {
        BRepBuilderAPI_MakeEdge edge(point(a),point(b));
        if (!edge.IsDone()) throw std::runtime_error("profile line failed");
        return cad->put(edge.Edge());
    });
}
int solvent_cad_circle(Cad* cad,const double* center,const double* normal,
    const double* x,double radius,double start,double end) noexcept {
    return guarded(cad,[&] {
        gp_Circ circle(gp_Ax2(point(center),direction(normal),direction(x)),radius);
        BRepBuilderAPI_MakeEdge edge(circle,start,end);
        if (!edge.IsDone()) throw std::runtime_error("profile arc failed");
        return cad->put(edge.Edge());
    });
}
int solvent_cad_face(Cad* cad,const int* edges,int count) noexcept {
    return guarded(cad,[&] {
        BRepBuilderAPI_MakeWire wire;
        for (int i=0;i<count;++i) {
            wire.Add(TopoDS::Edge(cad->at(edges[i])));
            if (!wire.IsDone()) throw std::runtime_error("profile wire failed");
        }
        BRepBuilderAPI_MakeFace face(wire.Wire(),true);
        if (!face.IsDone()) throw std::runtime_error("planar profile failed");
        return cad->put(face.Face());
    });
}
int solvent_cad_prism(Cad* cad,int face,const double* placement,const double* sweep) noexcept {
    return guarded(cad,[&] {
        gp_Trsf transform;
        transform.SetTranslation(vector(placement));
        BRepBuilderAPI_Transform moved(cad->at(face),transform,true);
        BRepPrimAPI_MakePrism body(moved.Shape(),vector(sweep),true);
        auto shape = body.Shape();
        validate(shape);
        return cad->put(shape);
    });
}
int solvent_cad_revolve(Cad* cad,int face,const double* origin,const double* axis,double angle) noexcept {
    return guarded(cad,[&] {
        auto dir = direction(axis);
        if (angle < 0) dir.Reverse();
        BRepPrimAPI_MakeRevol body(cad->at(face),gp_Ax1(point(origin),dir),std::abs(angle),true);
        auto shape = body.Shape();
        validate(shape);
        return cad->put(shape);
    });
}
int solvent_cad_boolean(Cad* cad,int a,int b,int cut) noexcept {
    return guarded(cad,[&] {
        TopoDS_Shape result;
        if (cut) {
            BRepAlgoAPI_Cut operation(cad->at(a),cad->at(b));
            if (!operation.IsDone()) throw std::runtime_error("Boolean cut failed");
            result = operation.Shape();
        } else {
            BRepAlgoAPI_Fuse operation(cad->at(a),cad->at(b));
            if (!operation.IsDone()) throw std::runtime_error("Boolean union failed");
            result = operation.Shape();
        }
        return cad->put(result);
    });
}
int solvent_cad_transform(Cad* cad,int source,const double* matrix) noexcept {
    return guarded(cad,[&] {
        gp_Trsf pose;
        pose.SetValues(matrix[0],matrix[1],matrix[2],matrix[3],
            matrix[4],matrix[5],matrix[6],matrix[7],matrix[8],matrix[9],matrix[10],matrix[11]);
        BRepBuilderAPI_Transform moved(cad->at(source),pose,true);
        auto result = moved.Shape();
        validate(result);
        return cad->put(result);
    });
}
int solvent_cad_bounds(Cad* cad,const int* ids,int count,double* bounds) noexcept {
    return guarded(cad,[&] {
        Bnd_Box box;
        for (int i=0;i<count;++i) BRepBndLib::Add(cad->at(ids[i]),box,false);
        if (box.IsVoid() || box.IsOpen()) throw std::runtime_error("no finite material extent");
        box.Get(bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5]);
        return 0;
    });
}
int solvent_cad_validate(Cad* cad,int id) noexcept {
    return guarded(cad,[&] { validate(cad->at(id)); return 0; });
}
int solvent_cad_step(Cad* cad,int id,const char* path) noexcept {
    return guarded(cad,[&] {
        // The CLI reserves stdout for its JSON/text report. Restore the stream
        // even if STEP construction throws; this host runs synchronously.
        struct LogStream {
            std::streambuf* old = std::cout.rdbuf(std::cerr.rdbuf());
            ~LogStream() { std::cout.rdbuf(old); }
        } log_stream;
        auto& shape = cad->at(id);
        validate(shape);
        STEPControl_Writer writer;
        if (writer.Transfer(shape,STEPControl_AsIs) != IFSelect_RetDone
            || writer.Write(path) != IFSelect_RetDone) throw std::runtime_error("STEP write failed");
        STEPControl_Reader reader;
        if (reader.ReadFile(path) != IFSelect_RetDone || !reader.TransferRoots())
            throw std::runtime_error("STEP reimport failed");
        auto imported = reader.OneShape();
        validate(imported);
        if (std::abs(volume(shape)-volume(imported)) > 1e-9+1e-7*std::abs(volume(shape)))
            throw std::runtime_error("STEP round trip changed solid volume");
        return 0;
    });
}
int solvent_cad_stl(Cad* cad,int id,const char* path) noexcept {
    return guarded(cad,[&] {
        auto& shape = cad->at(id);
        validate(shape);
        // Absolute millimetres, matching the native construction recipe. These
        // are tessellator controls, not an end-to-end geometry error certificate.
        BRepMesh_IncrementalMesh mesher(shape,0.01,false,0.2,false);
        if (!mesher.IsDone()) throw std::runtime_error("native tessellation failed");
        for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) {
            TopLoc_Location location;
            auto triangles = BRep_Tool::Triangulation(TopoDS::Face(it.Current()),location);
            if (triangles.IsNull() || triangles->NbTriangles() == 0)
                throw std::runtime_error("native tessellation omitted a face");
        }
        StlAPI_Writer writer;
        writer.ASCIIMode() = false;
        if (!writer.Write(shape,path)) throw std::runtime_error("STL write failed");
        return 0;
    });
}
}
