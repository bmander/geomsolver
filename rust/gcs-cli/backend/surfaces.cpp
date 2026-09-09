// Candidate surface fitting and native intersection/trim operations.
#include "occt.hpp"
#include <Approx_ParametrizationType.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt2d.hxx>
#include <cmath>
#include <sstream>

static gp_Pnt point(const double* p) { return gp_Pnt(p[0],p[1],p[2]); }
static gp_Pnt2d parameter(const Handle(Geom_Surface)& surface,double u,double v) {
    if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || u > 1 || v < 0 || v > 1)
        throw std::runtime_error("face query parameters must lie in [0,1]");
    if (surface.IsNull()) throw std::runtime_error("face has no surface");
    double u0,u1,v0,v1;
    surface->Bounds(u0,u1,v0,v1);
    if (!std::isfinite(u0) || !std::isfinite(u1) || !std::isfinite(v0) || !std::isfinite(v1))
        throw std::runtime_error("face query needs a finite surface domain");
    return gp_Pnt2d(u0+(u1-u0)*u,v0+(v1-v0)*v);
}

static TopoDS_Face valid_face(Cad* cad,int id) {
    const auto face = TopoDS::Face(cad->at(id));
    if (face.IsNull() || !BRepCheck_Analyzer(face).IsValid())
        throw std::runtime_error("invalid candidate face");
    return face;
}

static void surface_sample(const TopoDS_Face& face,const gp_Pnt2d& uv,bool oriented,double* output) {
    TopLoc_Location location;
    auto surface = BRep_Tool::Surface(face,location);
    if (surface.IsNull()) throw std::runtime_error("face has no surface");
    gp_Pnt p;
    gp_Vec du,dv;
    surface->D1(uv.X(),uv.Y(),p,du,dv);
    p.Transform(location.Transformation());
    du.Transform(location.Transformation()); dv.Transform(location.Transformation());
    gp_Vec n = du.Crossed(dv);
    if (n.SquareMagnitude() <= 0) throw std::runtime_error("singular face query");
    n.Normalize();
    if (oriented) {
        if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
        else if (face.Orientation() != TopAbs_FORWARD)
            throw std::runtime_error("face has no boundary orientation");
    }
    for (int k=1;k<=3;++k) {
        output[k-1] = p.Coord(k); output[k+2] = n.Coord(k);
        if (!std::isfinite(output[k-1]) || !std::isfinite(output[k+2]))
            throw std::runtime_error("nonfinite face query");
    }
}

static int membership(const TopoDS_Face& face,const gp_Pnt2d& uv,double tolerance) {
    BRepClass_FaceClassifier classify(face,uv,tolerance);
    switch (classify.State()) {
    case TopAbs_OUT: return 0;
    case TopAbs_IN: return 1;
    case TopAbs_ON: return 2;
    default: throw std::runtime_error("native face classification is unresolved");
    }
}

extern "C" {
// A regular contact chart, sampled in row-major (u, motion parameter) order.
// This creates a candidate face only. Solid closure and trimming remain separate.
int solvent_cad_bspline_face(Cad* cad,const double* points,int nu,int nv) noexcept {
    return guarded(cad,[&] {
        if (!points || nu < 2 || nv < 2 || nu > 512 || nv > 512)
            throw std::runtime_error("surface grid dimensions must lie in [2,512]");
        TColgp_Array2OfPnt grid(1,nu,1,nv);
        for (int i=0;i<nu;++i) for (int j=0;j<nv;++j) {
            const double* p = points+3*(i*nv+j);
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
                throw std::runtime_error("surface grid contains a nonfinite coordinate");
            grid.SetValue(i+1,j+1,point(p));
        }
        GeomAPI_PointsToBSplineSurface fit;
        fit.Interpolate(grid,Approx_IsoParametric,false);
        if (!fit.IsDone()) throw std::runtime_error("contact surface interpolation failed");
        BRepBuilderAPI_MakeFace face(fit.Surface(),1e-7);
        if (!face.IsDone() || !BRepCheck_Analyzer(face.Face()).IsValid())
            throw std::runtime_error("contact surface produced an invalid face");
        return cad->put(face.Face());
    });
}
// Evaluate the supporting surface, not membership in a trimmed face. Parameters
// span its bounds; the normal follows du cross dv, not material orientation.
int solvent_cad_surface_point(Cad* cad,int id,double u,double v,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("face query needs an output buffer");
        const auto face = TopoDS::Face(cad->at(id));
        surface_sample(face,parameter(BRep_Tool::Surface(face),u,v),false,output);
        return 0;
    });
}

// Query the bounded native face, including trim membership and orientation.
// Coordinates span this face's UV box, NOT the supporting-surface domain used
// by surface_point. Holes/outside regions return 0 and leave output untouched.
// Returns 1 inside or 2 on a trim, with position and oriented unit normal.
int solvent_cad_face_point(Cad* cad,int id,double u,double v,double tolerance,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output || !std::isfinite(tolerance) || tolerance <= 0)
            throw std::runtime_error("face query needs an output buffer and positive tolerance");
        if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || u > 1 || v < 0 || v > 1)
            throw std::runtime_error("face query parameters must lie in [0,1]");
        const auto face = TopoDS::Face(cad->at(id));
        double a,b,c,d;
        BRepTools::UVBounds(face,a,b,c,d);
        for (double x: {a,b,c,d}) if (!std::isfinite(x) || Precision::IsInfinite(x))
            throw std::runtime_error("face query needs finite trim bounds");
        if (a >= b || c >= d) throw std::runtime_error("degenerate face bounds");
        const gp_Pnt2d uv(a+(b-a)*u,c+(d-c)*v);
        const int state = membership(face,uv,tolerance);
        if (state) surface_sample(face,uv,true,output);
        return state;
    });
}

// Split one candidate by other faces. The result retains ALL source fragments;
// tool fragments are excluded. This performs no material-side selection, healing,
// fuzzy merging, or solid assembly. Inputs remain unchanged.
int solvent_cad_split_face(Cad* cad,int source,const int* tools,int count) noexcept {
    return guarded(cad,[&] {
        if (!tools || count < 1 || count > 1024)
            throw std::runtime_error("face split requires 1..1024 tool faces");
        TopTools_ListOfShape objects,cutters;
        objects.Append(valid_face(cad,source));
        for (int i=0;i<count;++i) cutters.Append(valid_face(cad,tools[i]));
        BRepAlgoAPI_Splitter split;
        split.SetArguments(objects); split.SetTools(cutters);
        split.SetNonDestructive(true); split.SetRunParallel(false);
        split.Build();
        if (!split.IsDone() || split.HasErrors() || split.HasWarnings()) {
            std::ostringstream message;
            message << "candidate face split failed: ";
            split.DumpErrors(message); split.DumpWarnings(message);
            throw std::runtime_error(message.str());
        }
        const auto result = split.Shape();
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(result,TopAbs_FACE,faces);
        if (faces.IsEmpty() || !BRepCheck_Analyzer(result).IsValid())
            throw std::runtime_error("face split produced no valid faces");
        return cad->put(result);
    });
}

// Count first with faces=null/capacity=0, then retrieve session-owned handles.
int solvent_cad_faces(Cad* cad,int source,int* output,int capacity) noexcept {
    return guarded(cad,[&] {
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(cad->at(source),TopAbs_FACE,faces);
        const int count = faces.Extent();
        if (!output && capacity == 0) return count;
        if (!output || capacity < count) throw std::runtime_error("face buffer is too small");
        for (int i=1;i<=count;++i) output[i-1] = cad->put(faces(i));
        return count;
    });
}

// Query native trim membership at the SAME normalized supporting-surface
// parameters as surface_point: 0=outside, 1=inside, 2=on a trimming edge.
// Normalization never switches to a fragment's smaller UV bounding rectangle.
// Tolerance is the native UV classifier tolerance, not an export error budget.
int solvent_cad_face_contains(Cad* cad,int id,double u,double v,double tolerance) noexcept {
    return guarded(cad,[&] {
        if (!std::isfinite(tolerance) || tolerance <= 0)
            throw std::runtime_error("face classification needs a positive tolerance");
        const auto face = TopoDS::Face(cad->at(id));
        const auto uv = parameter(BRep_Tool::Surface(face),u,v);
        return membership(face,uv,tolerance);
    });
}
}
