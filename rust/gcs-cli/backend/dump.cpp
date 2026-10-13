// A stored shape as JSON for the core's own B-rep (`gcs_core::brep::json`, phase 1 of
// docs/rust-kernel-plan.md): its vertices, its edges on their curves with their parameter
// ranges and ends, its faces on their surfaces with their loops of oriented edge uses, each use
// carrying its curve in the face's parameters. Geometry is written as the kernel holds it — every
// location applied, a trimmed curve or surface as its basis, a placement with its own three axes
// (a left-handed one as it is, for the reader to turn) — and a kind the core has no counterpart for
// converted to a B-spline exactly. Numbers are written with every digit a double has.
#include "occt.hpp"
#include "probe.hpp"
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomConvert.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom2d_Line.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Lin.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dConvert.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <cstdlib>
#include <sstream>

namespace {

void xyz(std::ostream& o,const gp_XYZ& v) { o << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']'; }

void frame(std::ostream& o,const gp_Ax3& a) {
    o << "\"frame\":{\"o\":"; xyz(o,a.Location().XYZ());
    o << ",\"x\":"; xyz(o,a.XDirection().XYZ());
    o << ",\"y\":"; xyz(o,a.YDirection().XYZ());
    o << ",\"z\":"; xyz(o,a.Direction().XYZ());
    o << '}';
}

void reals(std::ostream& o,const TColStd_Array1OfReal& a) {
    o << '[';
    for (int i=a.Lower();i<=a.Upper();++i) { if (i > a.Lower()) o << ','; o << a(i); }
    o << ']';
}

void curve(std::ostream& o,Handle(Geom_Curve) c) {
    while (auto t = Handle(Geom_TrimmedCurve)::DownCast(c)) c = t->BasisCurve();
    if (auto l = Handle(Geom_Line)::DownCast(c)) {
        o << "{\"kind\":\"line\",\"p\":"; xyz(o,l->Lin().Location().XYZ());
        o << ",\"d\":"; xyz(o,l->Lin().Direction().XYZ()); o << '}';
        return;
    }
    if (auto k = Handle(Geom_Circle)::DownCast(c)) {
        o << "{\"kind\":\"circle\","; frame(o,gp_Ax3(k->Position())); o << ",\"r\":" << k->Radius() << '}';
        return;
    }
    if (auto e = Handle(Geom_Ellipse)::DownCast(c)) {
        o << "{\"kind\":\"ellipse\","; frame(o,gp_Ax3(e->Position()));
        o << ",\"a\":" << e->MajorRadius() << ",\"b\":" << e->MinorRadius() << '}';
        return;
    }
    Handle(Geom_BSplineCurve) b = Handle(Geom_BSplineCurve)::DownCast(c);
    if (b.IsNull()) b = GeomConvert::CurveToBSplineCurve(c);
    else b = Handle(Geom_BSplineCurve)::DownCast(b->Copy());
    if (b->IsPeriodic()) b->SetNotPeriodic();
    TColStd_Array1OfReal knots(1,b->NbPoles()+b->Degree()+1);
    b->KnotSequence(knots);
    o << "{\"kind\":\"bspline\",\"degree\":" << b->Degree() << ",\"knots\":"; reals(o,knots);
    o << ",\"poles\":[";
    for (int i=1;i<=b->NbPoles();++i) { if (i > 1) o << ','; xyz(o,b->Pole(i).XYZ()); }
    o << ']';
    if (b->IsRational()) {
        o << ",\"weights\":[";
        for (int i=1;i<=b->NbPoles();++i) { if (i > 1) o << ','; o << b->Weight(i); }
        o << ']';
    }
    o << '}';
}

// A curve in a face's parameters, as the 3D curve kinds with z = 0: a line, or a B-spline.
void pcurve(std::ostream& o,Handle(Geom2d_Curve) c) {
    while (auto t = Handle(Geom2d_TrimmedCurve)::DownCast(c)) c = t->BasisCurve();
    if (auto l = Handle(Geom2d_Line)::DownCast(c)) {
        const gp_Pnt2d p = l->Lin2d().Location();
        const gp_Dir2d d = l->Lin2d().Direction();
        o << "{\"kind\":\"line\",\"p\":[" << p.X() << ',' << p.Y() << ",0],\"d\":[" << d.X() << ',' << d.Y() << ",0]}";
        return;
    }
    Handle(Geom2d_BSplineCurve) b = Handle(Geom2d_BSplineCurve)::DownCast(c);
    if (b.IsNull()) b = Geom2dConvert::CurveToBSplineCurve(c);
    else b = Handle(Geom2d_BSplineCurve)::DownCast(b->Copy());
    if (b->IsPeriodic()) b->SetNotPeriodic();
    TColStd_Array1OfReal knots(1,b->NbPoles()+b->Degree()+1);
    b->KnotSequence(knots);
    o << "{\"kind\":\"bspline\",\"degree\":" << b->Degree() << ",\"knots\":"; reals(o,knots);
    o << ",\"poles\":[";
    for (int i=1;i<=b->NbPoles();++i) { if (i > 1) o << ','; o << '[' << b->Pole(i).X() << ',' << b->Pole(i).Y() << ",0]"; }
    o << ']';
    if (b->IsRational()) {
        o << ",\"weights\":[";
        for (int i=1;i<=b->NbPoles();++i) { if (i > 1) o << ','; o << b->Weight(i); }
        o << ']';
    }
    o << '}';
}

void surface(std::ostream& o,Handle(Geom_Surface) s) {
    while (auto t = Handle(Geom_RectangularTrimmedSurface)::DownCast(s)) s = t->BasisSurface();
    if (auto p = Handle(Geom_Plane)::DownCast(s)) { o << "{\"kind\":\"plane\","; frame(o,p->Position()); o << '}'; return; }
    if (auto c = Handle(Geom_CylindricalSurface)::DownCast(s)) {
        o << "{\"kind\":\"cylinder\","; frame(o,c->Position()); o << ",\"r\":" << c->Radius() << '}';
        return;
    }
    if (auto c = Handle(Geom_ConicalSurface)::DownCast(s)) {
        o << "{\"kind\":\"cone\","; frame(o,c->Position());
        o << ",\"r\":" << c->RefRadius() << ",\"angle\":" << c->SemiAngle() << '}';
        return;
    }
    if (auto c = Handle(Geom_SphericalSurface)::DownCast(s)) {
        o << "{\"kind\":\"sphere\","; frame(o,c->Position()); o << ",\"r\":" << c->Radius() << '}';
        return;
    }
    if (auto c = Handle(Geom_ToroidalSurface)::DownCast(s)) {
        o << "{\"kind\":\"torus\","; frame(o,c->Position());
        o << ",\"big\":" << c->MajorRadius() << ",\"r\":" << c->MinorRadius() << '}';
        return;
    }
    Handle(Geom_BSplineSurface) b = Handle(Geom_BSplineSurface)::DownCast(s);
    if (b.IsNull()) b = GeomConvert::SurfaceToBSplineSurface(s);
    else b = Handle(Geom_BSplineSurface)::DownCast(b->Copy());
    if (b->IsUPeriodic()) b->SetUNotPeriodic();
    if (b->IsVPeriodic()) b->SetVNotPeriodic();
    TColStd_Array1OfReal uk(1,b->NbUPoles()+b->UDegree()+1), vk(1,b->NbVPoles()+b->VDegree()+1);
    b->UKnotSequence(uk); b->VKnotSequence(vk);
    o << "{\"kind\":\"bspline\",\"du\":" << b->UDegree() << ",\"dv\":" << b->VDegree() << ",\"uknots\":"; reals(o,uk);
    o << ",\"vknots\":"; reals(o,vk);
    // the poles by u, then v: row i the poles at u index i
    o << ",\"poles\":[";
    for (int i=1;i<=b->NbUPoles();++i) {
        if (i > 1) o << ',';
        o << '[';
        for (int j=1;j<=b->NbVPoles();++j) { if (j > 1) o << ','; xyz(o,b->Pole(i,j).XYZ()); }
        o << ']';
    }
    o << ']';
    if (b->IsURational() || b->IsVRational()) {
        o << ",\"weights\":[";
        for (int i=1;i<=b->NbUPoles();++i) {
            if (i > 1) o << ',';
            o << '[';
            for (int j=1;j<=b->NbVPoles();++j) { if (j > 1) o << ','; o << b->Weight(i,j); }
            o << ']';
        }
        o << ']';
    }
    o << '}';
}

std::string dump(const TopoDS_Shape& shape) {
    std::ostringstream o;
    o.precision(17);
    TopTools_IndexedMapOfShape vertices,edges;
    TopExp::MapShapes(shape,TopAbs_VERTEX,vertices);
    TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    o << "{\"vertices\":[";
    for (int i=1;i<=vertices.Extent();++i) {
        const TopoDS_Vertex v = TopoDS::Vertex(vertices(i));
        if (i > 1) o << ',';
        o << "{\"p\":"; xyz(o,BRep_Tool::Pnt(v).XYZ()); o << ",\"tol\":" << BRep_Tool::Tolerance(v) << '}';
    }
    o << "],\"edges\":[";
    for (int i=1;i<=edges.Extent();++i) {
        const TopoDS_Edge e = TopoDS::Edge(edges(i).Oriented(TopAbs_FORWARD));
        if (i > 1) o << ',';
        TopoDS_Vertex a,b;
        TopExp::Vertices(e,a,b);
        double t0,t1;
        BRep_Tool::Range(e,t0,t1);
        o << "{\"v\":[" << vertices.FindIndex(a)-1 << ',' << vertices.FindIndex(b)-1 << "],\"t\":[" << t0 << ',' << t1
          << "],\"tol\":" << BRep_Tool::Tolerance(e);
        if (BRep_Tool::Degenerated(e)) o << ",\"degenerate\":true";
        else {
            double f,l;
            const Handle(Geom_Curve) c = BRep_Tool::Curve(e,f,l);
            if (c.IsNull()) throw std::runtime_error("an edge has no 3D curve");
            o << ",\"curve\":"; curve(o,c);
        }
        o << '}';
    }
    o << "],\"faces\":[";
    bool first_face = true;
    for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) {
        // the face's loops and pcurves as it is stored, forward: a reversed face's are read the
        // other way round by the reader (a seam's pcurve is picked by the edge's orientation, which a
        // reversed face would turn once more)
        const bool reversed = it.Current().Orientation() == TopAbs_REVERSED;
        const TopoDS_Face face = TopoDS::Face(it.Current().Oriented(TopAbs_FORWARD));
        if (!first_face) o << ',';
        first_face = false;
        o << "{\"reversed\":" << (reversed ? "true" : "false") << ",\"surface\":";
        surface(o,BRep_Tool::Surface(face));
        const TopoDS_Wire outer = BRepTools::OuterWire(face);
        o << ",\"loops\":[";
        bool first_loop = true;
        for (TopExp_Explorer w(face,TopAbs_WIRE); w.More(); w.Next()) {
            const TopoDS_Wire wire = TopoDS::Wire(w.Current());
            if (!first_loop) o << ',';
            first_loop = false;
            o << "{\"outer\":" << (wire.IsSame(outer) ? "true" : "false") << ",\"uses\":[";
            bool first_use = true;
            // each use oriented as the wire holds it, which picks a seam's pcurve, in the wire's own
            // order: a wire explorer's chaining misreads a closed edge's orientation, so the reader
            // orders the uses by their vertices and parameters instead
            for (TopExp_Explorer x(wire,TopAbs_EDGE); x.More(); x.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(x.Current());
                if (!first_use) o << ',';
                first_use = false;
                double f,l;
                const Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e,face,f,l);
                if (pc.IsNull()) throw std::runtime_error("an edge has no curve in a face's parameters");
                o << "{\"edge\":" << edges.FindIndex(e)-1 << ",\"reversed\":"
                  << (e.Orientation() == TopAbs_REVERSED ? "true" : "false") << ",\"pcurve\":";
                pcurve(o,pc);
                o << '}';
            }
            o << "]}";
        }
        o << "]}";
    }
    o << "]";
    // the kernel's own volume, adaptive to the relative error `SOLVENT_BREP_VOLUME` names, for a
    // reader to compare with its own
    if (const char* eps = std::getenv("SOLVENT_BREP_VOLUME")) {
        GProp_GProps props;
        const double err = BRepGProp::VolumeProperties(shape,props,std::atof(eps),true);
        o << ",\"volume\":" << props.Mass() << ",\"volume_error\":" << err;
    }
    o << "}";
    return o.str();
}

}

extern "C" {
// The shape `id` as JSON (see above), in a buffer this thread keeps until its next call; null on
// failure, `solvent_cad_error` saying why.
const char* solvent_cad_brep_json(Cad* cad,int id) noexcept {
    SOLVENT_PROBE("solvent_cad_brep_json");
    thread_local std::string text;
    const int ok = guarded(cad,[&] { text = dump(cad->at(id)); return 0; });
    return ok < 0 ? nullptr : text.c_str();
}
}
