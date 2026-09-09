// Curves on native faces, for contact cuts without tangent face intersections.
#include "occt.hpp"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Curve.hxx>
#include <Precision.hxx>
#include <TColgp_HArray1OfPnt2d.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <algorithm>
#include <cmath>

struct FaceChart {
    TopoDS_Face face;
    Handle(Geom_Surface) surface;
    TopLoc_Location location;
    double a,b,c,d;
    FaceChart(Cad* cad,int id): face(TopoDS::Face(cad->at(id))) {
        surface = BRep_Tool::Surface(face,location);
        if (surface.IsNull()) throw std::runtime_error("face has no surface");
        BRepTools::UVBounds(face,a,b,c,d);
        for (double x: {a,b,c,d}) if (!std::isfinite(x) || Precision::IsInfinite(x))
            throw std::runtime_error("contact curve needs finite face bounds");
        if (a >= b || c >= d) throw std::runtime_error("degenerate face bounds");
    }
    gp_Pnt2d parameter(double u,double v) const {
        if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || u > 1 || v < 0 || v > 1)
            throw std::runtime_error("contact coordinates must lie in [0,1]");
        return gp_Pnt2d(a+(b-a)*u,c+(d-c)*v);
    }
};
static void tolerance(double value) {
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("contact curve needs a positive finite tolerance");
}

extern "C" {
// Only a full period across the finite face box supplies opposite seam aliases.
// Spatial incidence is checked separately when a trace actually reaches them.
int solvent_cad_face_seams(Cad* cad,int id,int* axes) noexcept {
    return guarded(cad,[&] {
        if (!axes) throw std::runtime_error("seam query needs an output buffer");
        const FaceChart chart(cad,id);
        axes[0] = chart.surface->IsUPeriodic() &&
            std::abs(chart.b-chart.a-chart.surface->UPeriod()) <= Precision::PConfusion();
        axes[1] = chart.surface->IsVPeriodic() &&
            std::abs(chart.d-chart.c-chart.surface->VPeriod()) <= Precision::PConfusion();
        return 0;
    });
}

// Inspect the actual spatial curve carried by an edge (including its location),
// with a normalized parameter over its finite range.
int solvent_cad_curve_point(Cad* cad,int id,double t,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output || !std::isfinite(t) || t < 0 || t > 1)
            throw std::runtime_error("edge query needs a buffer and parameter in [0,1]");
        TopLoc_Location location; double a,b;
        const auto curve = BRep_Tool::Curve(TopoDS::Edge(cad->at(id)),location,a,b);
        if (curve.IsNull() || !std::isfinite(a) || !std::isfinite(b) || a >= b)
            throw std::runtime_error("edge has no finite spatial curve");
        auto p = curve->Value(a+(b-a)*t);
        p.Transform(location.Transformation());
        for (int k=1;k<=3;++k) {
            if (!std::isfinite(p.Coord(k))) throw std::runtime_error("nonfinite curve point");
            output[k-1] = p.Coord(k);
        }
        return 0;
    });
}
// Project a world-mm point into the face's finite UV box. Return 0 if no point
// within the requested spatial distance belongs to its trims; output untouched.
// Return 1 with normalized u,v and measured distance. This is incidence, not a
// choice of a globally continuous branch across seams or singularities.
int solvent_cad_face_parameters(Cad* cad,int id,const double* point,double distance,double* output) noexcept {
    return guarded(cad,[&] {
        tolerance(distance);
        if (!point || !output) throw std::runtime_error("projection needs input and output buffers");
        for (int k=0;k<3;++k) if (!std::isfinite(point[k])) throw std::runtime_error("nonfinite projection point");
        const FaceChart chart(cad,id);
        gp_Pnt p(point[0],point[1],point[2]);
        p.Transform(chart.location.Transformation().Inverted());
        // Project on the support first, then classify its actual trims. Restricting
        // the extrema search to a fragment's UV box can report failure merely
        // because the ordinary orthogonal foot belongs to another fragment.
        GeomAPI_ProjectPointOnSurf project(p,chart.surface,Precision::PConfusion());
        if (!project.IsDone()) throw std::runtime_error("native face projection failed");
        double best = distance,u = 0,v = 0;
        bool found = false;
        for (int i=1;i<=project.NbPoints();++i) {
            double a,b; project.Parameters(i,a,b);
            if (chart.surface->IsUPeriodic()) {
                const double period = chart.surface->UPeriod();
                a += std::round(((chart.a+chart.b)*0.5-a)/period)*period;
            }
            if (chart.surface->IsVPeriodic()) {
                const double period = chart.surface->VPeriod();
                b += std::round(((chart.c+chart.d)*0.5-b)/period)*period;
            }
            // The projector can overshoot an exact trim-box endpoint by a few
            // ulps. Return an actual in-box point and remeasure its incidence;
            // never reuse the distance of the unclamped projection.
            a = std::clamp(a,chart.a,chart.b); b = std::clamp(b,chart.c,chart.d);
            const double gap = p.Distance(chart.surface->Value(a,b));
            if (!std::isfinite(gap) || gap > best) continue;
            BRepClass_FaceClassifier classify(chart.face,gp_Pnt2d(a,b),Precision::PConfusion());
            if (classify.State() == TopAbs_UNKNOWN) throw std::runtime_error("projection trim membership unresolved");
            if (classify.State() == TopAbs_OUT) continue;
            u = (a-chart.a)/(chart.b-chart.a); v = (b-chart.c)/(chart.d-chart.c);
            best = gap; found = true;
        }
        if (!found) return 0;
        output[0] = u; output[1] = v; output[2] = best;
        return 1;
    });
}

// The face's outward unit normal at the projection of a point onto its support,
// with the projection's distance. No trim test: a point on a face's boundary
// edge is on the face, whatever a classifier says within its tolerance.
int solvent_cad_face_normal(Cad* cad,int id,const double* point,double* output) noexcept {
    return guarded(cad,[&] {
        if (!point || !output) throw std::runtime_error("normal query needs input and output buffers");
        const FaceChart chart(cad,id);
        gp_Pnt p(point[0],point[1],point[2]);
        p.Transform(chart.location.Transformation().Inverted());
        GeomAPI_ProjectPointOnSurf project(p,chart.surface,Precision::PConfusion());
        if (!project.IsDone() || project.NbPoints() < 1) throw std::runtime_error("native face projection failed");
        double a,b; project.LowerDistanceParameters(a,b);
        if (chart.surface->IsUPeriodic()) { const double period = chart.surface->UPeriod(); a += std::round(((chart.a+chart.b)*0.5-a)/period)*period; }
        if (chart.surface->IsVPeriodic()) { const double period = chart.surface->VPeriod(); b += std::round(((chart.c+chart.d)*0.5-b)/period)*period; }
        gp_Pnt q; gp_Vec du,dv;
        chart.surface->D1(a,b,q,du,dv);
        gp_Vec n = du.Crossed(dv);
        if (n.SquareMagnitude() <= 0) throw std::runtime_error("singular face normal");
        n.Normalize();
        if (chart.face.Orientation() == TopAbs_REVERSED) n.Reverse();
        else if (chart.face.Orientation() != TopAbs_FORWARD) throw std::runtime_error("face has no boundary orientation");
        n.Transform(chart.location.Transformation());
        q.Transform(chart.location.Transformation());
        for (int k=1;k<=3;++k) output[k-1] = n.Coord(k);
        output[3] = q.Distance(gp_Pnt(point[0],point[1],point[2]));
        return 0;
    });
}

// Interpolate a curve in the face's finite UV chart and build its spatial edge.
// UV fitting is not a certificate of the original contact curve's spatial error.
// Closed input omits a duplicate final point; open endpoints must reach trims.
int solvent_cad_pcurve(Cad* cad,int id,const double* points,int count,int closed,double distance) noexcept {
    return guarded(cad,[&] {
        tolerance(distance);
        if (!points || count < (closed ? 3 : 2) || count > 4096 || (closed != 0 && closed != 1))
            throw std::runtime_error("invalid contact curve samples");
        const FaceChart chart(cad,id);
        Handle(TColgp_HArray1OfPnt2d) samples = new TColgp_HArray1OfPnt2d(1,count);
        for (int i=0;i<count;++i) {
            const auto uv = chart.parameter(points[2*i],points[2*i+1]);
            BRepClass_FaceClassifier classify(chart.face,uv,Precision::PConfusion());
            if (classify.State() != TopAbs_IN && classify.State() != TopAbs_ON)
                throw std::runtime_error("contact curve sample leaves face trims");
            if (!closed && (i == 0 || i == count-1) && classify.State() != TopAbs_ON)
                throw std::runtime_error("open contact curve must end on face trims");
            if (i > 0 && uv.Distance(samples->Value(i)) <= Precision::PConfusion())
                throw std::runtime_error("contact curve has coincident samples");
            samples->SetValue(i+1,uv);
        }
        Geom2dAPI_Interpolate fit(samples,closed != 0,Precision::PConfusion());
        fit.Perform();
        if (!fit.IsDone()) throw std::runtime_error("contact pcurve interpolation failed");
        BRepBuilderAPI_MakeEdge make(fit.Curve(),chart.surface);
        if (!make.IsDone()) throw std::runtime_error("contact pcurve edge construction failed");
        auto edge = make.Edge();
        edge.Move(chart.location);
        if (!BRepLib::BuildCurve3d(edge,distance) || !BRepCheck_Analyzer(edge).IsValid())
            throw std::runtime_error("contact pcurve has no valid spatial edge");
        return cad->put(edge);
    });
}

// All fragments are retained. No orientation/visibility selection is implied by
// the curve direction. The common splitter preserves inputs and constructs their
// intersections with native trimming edges before rebuilding face wires.
int solvent_cad_split_pcurves(Cad* cad,int id,const int* edges,int count) noexcept {
    return guarded(cad,[&] {
        if (!edges || count < 1 || count > 1024) throw std::runtime_error("split requires 1..1024 contact edges");
        const FaceChart chart(cad,id);
        TopTools_ListOfShape tools;
        for (int i=0;i<count;++i) {
            const auto edge = TopoDS::Edge(cad->at(edges[i]));
            double a,b;
            if (BRep_Tool::CurveOnSurface(edge,chart.face,a,b).IsNull())
                throw std::runtime_error("contact edge is not attached to this face");
            tools.Append(edge);
        }
        const auto result = split_face(chart.face,tools);
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(result,TopAbs_FACE,faces);
        if (faces.Extent() < 2) throw std::runtime_error("contact curves did not separate the face");
        return cad->put(result);
    });
}
}
