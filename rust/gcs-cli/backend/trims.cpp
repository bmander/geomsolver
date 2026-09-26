#include <limits>
// Queries of native faces and edges: seams, curve points, projections and normals.
#include "occt.hpp"
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Curve.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
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
};
// A projector onto a face's whole support, reused across many points.
struct FaceProjector {
    FaceChart chart;
    GeomAPI_ProjectPointOnSurf project;
    // The whole support's bounds, as the single-point constructor reads them itself.
    FaceProjector(Cad* cad,int id): chart(cad,id) {
        double u0,u1,v0,v1;
        chart.surface->Bounds(u0,u1,v0,v1);
        project.Init(chart.surface,u0,u1,v0,v1,Precision::PConfusion());
    }
};

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
// For many points, the support surface's unit normal (du x dv, no orientation) at each
// point's nearest foot, and the distance to it: rows of nx, ny, nz, distance, with a NaN
// distance where the projection finds no foot. One projector serves them all.
int solvent_cad_surface_feet(Cad* cad,int id,const double* points,int count,double* output) noexcept {
    return guarded(cad,[&] {
        if (!points || !output || count < 0) throw std::runtime_error("surface feet need input and output buffers");
        FaceProjector projector(cad,id);
        const auto& chart = projector.chart;
        for (int i=0;i<count;++i) {
            double* row = output+4*i;
            gp_Pnt p(points[3*i],points[3*i+1],points[3*i+2]);
            p.Transform(chart.location.Transformation().Inverted());
            projector.project.Perform(p);
            if (!projector.project.IsDone() || projector.project.NbPoints() < 1) {
                row[0] = row[1] = row[2] = 0; row[3] = std::numeric_limits<double>::quiet_NaN(); continue;
            }
            double a,b; projector.project.LowerDistanceParameters(a,b);
            gp_Pnt q; gp_Vec du,dv;
            chart.surface->D1(a,b,q,du,dv);
            gp_Vec n = du.Crossed(dv);
            const double length = n.Magnitude();
            if (length > 0) n.Divide(length);
            n.Transform(chart.location.Transformation());
            row[0] = n.X(); row[1] = n.Y(); row[2] = n.Z();
            row[3] = projector.project.LowerDistance();
        }
        return 0;
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

}
