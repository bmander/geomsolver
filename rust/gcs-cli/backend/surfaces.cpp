// Surface fitting through a grid, and bounded face queries.
#include "occt.hpp"
#include "probe.hpp"
#include <Approx_ParametrizationType.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <BRepAdaptor_Surface.hxx>
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

static gp_Pnt point(const double* p) { return gp_Pnt(p[0],p[1],p[2]); }
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
// Parametrization 0 is uniform (isoparametric), 1 chord length, 2 centripetal.
// Uniform parameters overshoot where sample spacing changes abruptly.
int solvent_cad_bspline_face_with(Cad* cad,const double* points,int nu,int nv,int parametrization) noexcept {
    SOLVENT_PROBE("solvent_cad_bspline_face_with");
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
        const Approx_ParametrizationType type = parametrization == 1 ? Approx_ChordLength
            : parametrization == 2 ? Approx_Centripetal : Approx_IsoParametric;
        fit.Interpolate(grid,type,false);
        if (!fit.IsDone()) throw std::runtime_error("contact surface interpolation failed");
        BRepBuilderAPI_MakeFace face(fit.Surface(),1e-7);
        if (!face.IsDone() || !BRepCheck_Analyzer(face.Face()).IsValid())
            throw std::runtime_error("contact surface produced an invalid face");
        return cad->put(face.Face());
    });
}
// Query the bounded native face, including trim membership and orientation.
// Coordinates span this face's UV box, not its supporting surface's domain.
// Holes/outside regions return 0 and leave output untouched.
// Returns 1 inside or 2 on a trim, with position and oriented unit normal.
int solvent_cad_face_point(Cad* cad,int id,double u,double v,double tolerance,double* output) noexcept {
    SOLVENT_PROBE("solvent_cad_face_point");
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

// Count first with faces=null/capacity=0, then retrieve session-owned handles.
int solvent_cad_faces(Cad* cad,int source,int* output,int capacity) noexcept {
    SOLVENT_PROBE("solvent_cad_faces");
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
// The kind of a face's supporting surface: GeomAbs_SurfaceType's order (0 plane, 1 cylinder,
// 2 cone, 3 sphere, 4 torus, 5 Bezier, 6 B-spline, 7 revolution, 8 extrusion, 9 offset, 10 other).
int solvent_cad_face_kind(Cad* cad,int id) noexcept {
    SOLVENT_PROBE("solvent_cad_face_kind");
    return guarded(cad,[&] {
        const auto face = TopoDS::Face(cad->at(id));
        return static_cast<int>(BRepAdaptor_Surface(face,false).GetType());
    });
}
// A face's supporting surface on an even nu x nv grid over its UV box, row-major with u the
// first: six doubles a point, position and oriented unit normal. No trim test, so it reads a
// fitted sheet's whole chart quickly.
int solvent_cad_surface_grid(Cad* cad,int id,int nu,int nv,double* output) noexcept {
    SOLVENT_PROBE("solvent_cad_surface_grid");
    return guarded(cad,[&] {
        if (!output || nu < 2 || nv < 2) throw std::runtime_error("surface grid needs an output buffer and two points a side");
        const auto face = TopoDS::Face(cad->at(id));
        double a,b,c,d;
        BRepTools::UVBounds(face,a,b,c,d);
        for (double x: {a,b,c,d}) if (!std::isfinite(x) || Precision::IsInfinite(x))
            throw std::runtime_error("surface grid needs finite face bounds");
        for (int i=0;i<nu;++i) for (int j=0;j<nv;++j)
            surface_sample(face,gp_Pnt2d(a+(b-a)*i/(nu-1),c+(d-c)*j/(nv-1)),true,output+6*(i*nv+j));
        return 0;
    });
}
}
