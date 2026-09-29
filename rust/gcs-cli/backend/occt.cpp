// Native solid construction and export.
#include "occt.hpp"
#include <BRepCheck.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepCheck_ListOfStatus.hxx>
#include <sstream>
#include <BRepGProp.hxx>
#include <BRepGProp_Domain.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepGProp_Vinert.hxx>
#include <OSD_Parallel.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <NCollection_DataMap.hxx>
#include <algorithm>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepTools.hxx>
#include <Geom_Surface.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom2d_Curve.hxx>
#include <ElCLib.hxx>
#include <gp_Lin.hxx>
#include <Extrema_GenLocateExtPS.hxx>
#include <Extrema_POnSurf.hxx>
#include <Precision.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StepData_StepModel.hxx>
#include <StepData_StepWriter.hxx>
#include <StepData_WriterLib.hxx>
#include <StepData_Protocol.hxx>
#include <XSControl_WorkSession.hxx>
#include <thread>
#include <Standard_Failure.hxx>
#include <StlAPI_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static gp_Pnt point(const double* p) { return gp_Pnt(p[0],p[1],p[2]); }
static gp_Dir direction(const double* p) { return gp_Dir(p[0],p[1],p[2]); }
static gp_Vec vector(const double* p) { return gp_Vec(p[0],p[1],p[2]); }
// Adaptive quadrature: the fixed rule is off by parts per thousand on spline faces. It is
// `BRepGProp::VolumeProperties(shape, props, 1e-9)` — each FORWARD or REVERSED face's flux about
// the mean of the shape's vertices, summed in the explorer's order — with the faces integrated on
// every core (each face's integration is independent of the others'), so the sum is the same number.
double volume(const TopoDS_Shape& shape) {
    // the kernel's system location: the mean of the shape's vertices as its explorer lists them
    gp_XYZ sum(0,0,0);
    int count = 0;
    for (TopExp_Explorer it(shape,TopAbs_VERTEX); it.More(); it.Next(),++count) sum += BRep_Tool::Pnt(TopoDS::Vertex(it.Current())).XYZ();
    if (count > 0) sum /= count;
    std::vector<TopoDS_Face> faces;
    for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) {
        const TopAbs_Orientation o = it.Current().Orientation();
        if (o == TopAbs_FORWARD || o == TopAbs_REVERSED) faces.push_back(TopoDS::Face(it.Current()));
    }
    const double total = flux(faces,gp_Pnt(sum));
    // `SOLVENT_VOLUME_CHECK`: the kernel's own serial sum as well, which must be the same number.
    if (std::getenv("SOLVENT_VOLUME_CHECK")) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape,props,1e-9,false,false);
        if (props.Mass() != total) {
            fprintf(stderr,"volume: %.17g against the kernel's %.17g\n",total,props.Mass());
            throw std::runtime_error("the parallel volume differs from the kernel's");
        }
    }
    return total;
}
// The faces' flux about `origin`, a third of the integral of (x - origin)·n over them: the
// volume they bound when they close, each face integrated adaptively to 1e-9 relative on its own
// core and the whole summed in the faces' order.
double flux(const std::vector<TopoDS_Face>& faces,const gp_Pnt& origin) {
    std::vector<double> mass(faces.size(),0.);
    OSD_Parallel::For(0,static_cast<int>(faces.size()),[&](int i) {
        BRepGProp_Face face;
        face.Load(faces[static_cast<size_t>(i)]);
        BRepGProp_Vinert flux;
        flux.SetLocation(origin);
        if (TopoDS_Iterator(faces[static_cast<size_t>(i)]).More()) {
            BRepGProp_Domain domain(faces[static_cast<size_t>(i)]);
            flux.Perform(face,domain,1e-9);
        } else flux.Perform(face,1e-9);
        mass[static_cast<size_t>(i)] = flux.Mass();
    });
    double total = 0;
    for (const double m: mass) total += m;
    return total;
}
// A shape's area, `BRepGProp::SurfaceProperties`' on each face, the faces on every core.
double area(const TopoDS_Shape& shape) {
    std::vector<TopoDS_Face> faces;
    for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) faces.push_back(TopoDS::Face(it.Current()));
    std::vector<double> each(faces.size(),0.);
    OSD_Parallel::For(0,static_cast<int>(faces.size()),[&](int i) {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(faces[static_cast<size_t>(i)],props);
        each[static_cast<size_t>(i)] = props.Mass();
    });
    double total = 0;
    for (const double a: each) total += a;
    return total;
}
// The kernel's checker. (Its parallel mode is not used: in OCCT 7.9 it calls a valid pinion
// invalid, and not every time.)
bool valid(const TopoDS_Shape& shape) { return BRepCheck_Analyzer(shape).IsValid(); }

// The kernel's checker over one closed solid of one shell, a face at a time on every core: each
// face checked in its own analyzer (the face, its wires, and its edges and vertices on it), and
// the shell closed and oriented — every edge but a degenerate one used once forward and once
// reversed by the faces as the solid orients them. What the whole analyzer adds on one shell is
// its closure and orientation, which this asks directly; `SOLVENT_FULL_CHECK` runs the whole one.
bool valid_solid(const TopoDS_Shape& shape) {
    if (std::getenv("SOLVENT_FULL_CHECK")) return valid(shape);
    int shells = 0;
    for (TopExp_Explorer it(shape,TopAbs_SHELL); it.More(); it.Next()) ++shells;
    if (shells != 1 || shape.ShapeType() != TopAbs_SOLID) return valid(shape);
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape,TopAbs_FACE,faces);
    std::vector<char> ok(static_cast<size_t>(faces.Extent()),0);
    OSD_Parallel::For(0,faces.Extent(),[&](int i) { ok[static_cast<size_t>(i)] = BRepCheck_Analyzer(faces(i+1)).IsValid(); });
    if (std::find(ok.begin(),ok.end(),0) != ok.end()) return false;
    NCollection_DataMap<TopoDS_Shape,std::pair<int,int>,TopTools_ShapeMapHasher> uses;
    for (TopExp_Explorer f(shape,TopAbs_FACE); f.More(); f.Next()) for (TopExp_Explorer e(f.Current(),TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(e.Current());
        if (BRep_Tool::Degenerated(edge)) continue;
        if (edge.Orientation() != TopAbs_FORWARD && edge.Orientation() != TopAbs_REVERSED) return false;
        const TopoDS_Shape key = edge.Oriented(TopAbs_FORWARD);
        if (!uses.IsBound(key)) uses.Bind(key,{0,0});
        auto& count = uses.ChangeFind(key);
        (edge.Orientation() == TopAbs_FORWARD ? count.first : count.second) += 1;
    }
    for (NCollection_DataMap<TopoDS_Shape,std::pair<int,int>,TopTools_ShapeMapHasher>::Iterator it(uses); it.More(); it.Next())
        if (it.Value().first != 1 || it.Value().second != 1) return false;
    return true;
}
// Name the first invalid sub-shape and its statuses, so a failed construction
// says which face, edge or vertex the kernel objects to.
std::string invalidity(const TopoDS_Shape& shape) {
    BRepCheck_Analyzer analyzer(shape);
    std::ostringstream message;
    for (TopAbs_ShapeEnum kind: {TopAbs_SOLID,TopAbs_SHELL,TopAbs_FACE,TopAbs_WIRE,TopAbs_EDGE,TopAbs_VERTEX}) {
        int index = 0;
        for (TopExp_Explorer it(shape,kind); it.More(); it.Next(), ++index) {
            const auto result = analyzer.Result(it.Current());
            if (result.IsNull()) continue;
            bool bad = false;
            for (const auto status: result->Status()) if (status != BRepCheck_NoError) bad = true;
            if (!bad) continue;
            message << " " << (kind == TopAbs_SOLID ? "solid" : kind == TopAbs_SHELL ? "shell" : kind == TopAbs_FACE ? "face"
                : kind == TopAbs_WIRE ? "wire" : kind == TopAbs_EDGE ? "edge" : "vertex") << " " << index << ":";
            // The kernel's own name for each status (BRepCheck::Print ends it with a newline).
            for (const auto status: result->Status()) if (status != BRepCheck_NoError) {
                std::ostringstream name; BRepCheck::Print(status,name);
                std::string text = name.str();
                while (!text.empty() && (text.back() == '\n' || text.back() == ' ')) text.pop_back();
                message << " " << text;
            }
            return message.str();
        }
    }
    return message.str();
}
double validate(TopoDS_Shape& shape,TopTools_DataMapOfShapeReal* record,bool checked,double known) {
    if (shape.IsNull()) throw std::runtime_error("native solid is null");
    if (!checked && !valid(shape))
        throw std::runtime_error("native solid is invalid:"+invalidity(shape));
    if (shape.ShapeType() == TopAbs_SOLID) {
        auto solid = TopoDS::Solid(shape);
        if (!BRepLib::OrientClosedSolid(solid)) throw std::runtime_error("solid is open");
        shape = solid;
    }
    int count = 0;
    double total = 0;
    for (TopExp_Explorer it(shape,TopAbs_SOLID); it.More(); it.Next()) {
        if (!std::isnan(known) && count > 0) throw std::runtime_error("a volume known for one solid given for several");
        double v = std::isnan(known) ? volume(it.Current()) : known;
        if (!std::isfinite(v) || v <= 0) throw std::runtime_error("solid has no positive volume");
        if (record) record->Bind(it.Current(),v);
        total += v;
        ++count;
    }
    if (!count) throw std::runtime_error("operation produced no solid");
    return total;
}

void Cad::validated(int id) {
    {
        const std::lock_guard<std::mutex> hold(lock);
        if (valid.at(static_cast<size_t>(id))) return;
    }
    // A single solid's volume is what `validate` just measured.
    TopoDS_Shape& shape = at(id);
    const double v = validate(shape);
    const std::lock_guard<std::mutex> hold(lock);
    if (shape.ShapeType() == TopAbs_SOLID) volumes[static_cast<size_t>(id)] = v;
    valid[static_cast<size_t>(id)] = 1;
}
double Cad::volume_of(int id) {
    {
        const std::lock_guard<std::mutex> hold(lock);
        const double v = volumes.at(static_cast<size_t>(id));
        if (!std::isnan(v)) return v;
    }
    const double v = volume(at(id));
    const std::lock_guard<std::mutex> hold(lock);
    volumes[static_cast<size_t>(id)] = v;
    return v;
}
void Cad::record(const TopTools_DataMapOfShapeReal& volumes) {
    const std::lock_guard<std::mutex> hold(lock);
    for (TopTools_DataMapOfShapeReal::Iterator it(volumes); it.More(); it.Next()) measured.Bind(it.Key(),it.Value());
}
bool Cad::recorded(const TopoDS_Shape& solid,double& volume) {
    const std::lock_guard<std::mutex> hold(lock);
    if (!measured.IsBound(solid)) return false;
    volume = measured.Find(solid);
    return true;
}
std::string& last_error() { static thread_local std::string error; return error; }
// A Boolean's intersections on every core (`SOLVENT_PARALLEL_BOOLEANS=off`: one): the cutters of a
// body's sweeps are made, and the blank's clearance of them asked, side by side and each in parallel.
bool parallel_booleans() {
    static const bool on = [] { const char* v = std::getenv("SOLVENT_PARALLEL_BOOLEANS"); return !v || std::string(v) != "off"; }();
    return on;
}

// The chordal sag a meshed face actually has: over its triangles, the largest distance from a
// point linear in a triangle (its centroid, its edges' midpoints) to the face's surface, found by a
// local search from the surface parameters linear in the triangle; where that search fails, the
// distance to the surface at those parameters, which is no smaller. Raises `worst` to it, with
// where the linear point is.
void face_sag(const TopoDS_Face& face,double& worst,gp_Pnt& at) {
    TopLoc_Location location,placed;
    auto triangles = BRep_Tool::Triangulation(face,location);
    if (triangles.IsNull() || triangles->NbTriangles() == 0) throw std::runtime_error("mesh sag of an unmeshed face");
    if (!triangles->HasUVNodes()) throw std::runtime_error("a face's triangulation has no surface parameters");
    auto surface = BRep_Tool::Surface(face,placed);
    GeomAdaptor_Surface adaptor(surface);
    Extrema_GenLocateExtPS local(adaptor,Precision::PConfusion(),Precision::PConfusion());
    const gp_Trsf mesh = location.Transformation(),support = placed.Transformation(),back = support.Inverted();
    for (int t=1;t<=triangles->NbTriangles();++t) {
        int n[3]; triangles->Triangle(t).Get(n[0],n[1],n[2]);
        gp_Pnt p[3]; gp_Pnt2d uv[3];
        for (int k=0;k<3;++k) { p[k] = triangles->Node(n[k]).Transformed(mesh); uv[k] = triangles->UVNode(n[k]); }
        const double weights[4][3] = {{1./3,1./3,1./3},{0.5,0.5,0},{0,0.5,0.5},{0.5,0,0.5}};
        for (const auto& w: weights) {
            gp_XYZ linear(0,0,0); gp_XY param(0,0);
            for (int k=0;k<3;++k) { linear += w[k]*p[k].XYZ(); param += w[k]*uv[k].XY(); }
            const gp_Pnt point(linear),local_point = point.Transformed(back);
            double d = surface->Value(param.X(),param.Y()).Distance(local_point);
            try {
                local.Perform(local_point,param.X(),param.Y());
                if (local.IsDone()) d = std::min(d,std::sqrt(local.SquareDistance()));
            } catch (const Standard_Failure&) {}
            if (d > worst) { worst = d; at = point; }
        }
    }
}
// Whether `g` is `f` turned by `turn` as data, both faces on B-splines: the same orientation,
// degrees, knots and multiplicities, `g`'s poles within `tolerance` (mm) of `f`'s turned and its
// weights the same, and as many edges in the same order and orientation, each edge's pcurve taking
// the same points of the parameter plane over the same range (each to its precision in a file,
// below). A turn about a line keeps the flux
// about any point of it, so `g` then bounds with the flux `f` does.
static bool turned_spline_face(const TopoDS_Face& f,const TopoDS_Face& g,const gp_Trsf& turn,double tolerance) {
    if (f.Orientation() != g.Orientation()) return false;
    const auto x = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(f));
    const auto y = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(g));
    if (x.IsNull() || y.IsNull()) return false;
    const auto near = [](double p,double q) { return std::abs(p-q) <= 1e-12*(1+std::abs(p)); };
    if (x->UDegree() != y->UDegree() || x->VDegree() != y->VDegree() || x->NbUPoles() != y->NbUPoles()
        || x->NbVPoles() != y->NbVPoles() || x->NbUKnots() != y->NbUKnots() || x->NbVKnots() != y->NbVKnots()
        || x->IsURational() != y->IsURational() || x->IsVRational() != y->IsVRational()) return false;
    for (int k=1;k<=x->NbUKnots();++k) if (!near(x->UKnot(k),y->UKnot(k)) || x->UMultiplicity(k) != y->UMultiplicity(k)) return false;
    for (int k=1;k<=x->NbVKnots();++k) if (!near(x->VKnot(k),y->VKnot(k)) || x->VMultiplicity(k) != y->VMultiplicity(k)) return false;
    for (int m=1;m<=x->NbUPoles();++m) for (int n=1;n<=x->NbVPoles();++n)
        if (x->Pole(m,n).Transformed(turn).Distance(y->Pole(m,n)) > tolerance || !near(x->Weight(m,n),y->Weight(m,n))) return false;
    std::vector<TopoDS_Edge> e,d;
    for (TopExp_Explorer it(f,TopAbs_EDGE); it.More(); it.Next()) e.push_back(TopoDS::Edge(it.Current()));
    for (TopExp_Explorer it(g,TopAbs_EDGE); it.More(); it.Next()) d.push_back(TopoDS::Edge(it.Current()));
    if (e.size() != d.size()) return false;
    for (size_t k=0;k<e.size();++k) {
        if (e[k].Orientation() != d[k].Orientation()) return false;
        double e0,e1,d0,d1;
        const auto c = BRep_Tool::CurveOnSurface(e[k],f,e0,e1),h = BRep_Tool::CurveOnSurface(d[k],g,d0,d1);
        // (a copy's edge ranges and pcurves are its own, set by the sewing and the writer within
        // their precision: a billionth of the range, a ten-billionth of the parameter plane)
        if (c.IsNull() || h.IsNull() || std::abs(e0-d0) > 1e-9*(1+std::abs(e1-e0)) || std::abs(e1-d1) > 1e-9*(1+std::abs(e1-e0)))
            return false;
        for (int m=0;m<=4;++m) {
            const double at = e0+(e1-e0)*m/4;
            if (c->Value(at).Distance(h->Value(at)) > 1e-10) return false;
        }
    }
    return true;
}

// The volume of a solid that is `copies` turns of one sector about `axis` (a pattern, as a STEP
// file's reading of one): each face on a B-spline that is an earlier such face turned about the axis
// (`turned_spline_face`, within `tolerance` mm) bounds with that face's flux about a point of the
// axis, and every other face is measured. The same number as measuring every face, to the
// integration's precision, without integrating one sheet a copy.
static double patterned_volume(const TopoDS_Shape& shape,const gp_Ax1& axis,int copies,double tolerance) {
    std::vector<TopoDS_Face> faces;
    for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) {
        const TopAbs_Orientation o = it.Current().Orientation();
        if (o == TopAbs_FORWARD || o == TopAbs_REVERSED) faces.push_back(TopoDS::Face(it.Current()));
    }
    // Each face on a B-spline paired with the first one it is a turn of, where there is one.
    std::vector<int> twin(faces.size(),-1);
    std::vector<size_t> firsts;
    const double pitch = 2*M_PI/copies;
    const gp_Lin line(axis);
    for (size_t i=0;i<faces.size();++i) {
        const auto y = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(faces[i]));
        if (y.IsNull()) continue;
        for (const size_t j: firsts) {
            const auto x = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(faces[j]));
            if (x->NbUPoles() != y->NbUPoles() || x->NbVPoles() != y->NbVPoles()) continue;
            // the turn taking the first pole to the other's, a whole number of pitches
            const gp_Pnt p = x->Pole(1,1),q = y->Pole(1,1);
            const gp_Vec a(axis.Direction());
            const gp_Vec rp = gp_Vec(ElCLib::Value(ElCLib::Parameter(line,p),line),p),rq = gp_Vec(ElCLib::Value(ElCLib::Parameter(line,q),line),q);
            if (rp.Magnitude() <= tolerance || rq.Magnitude() <= tolerance) continue;
            const double angle = std::atan2(a.Dot(rp.Crossed(rq)),rp.Dot(rq));
            const double k = std::round(angle/pitch);
            if (std::abs(angle-k*pitch) > 1e-6) continue;
            gp_Trsf turn;
            turn.SetRotation(axis,k*pitch);
            if (turned_spline_face(faces[j],faces[i],turn,tolerance)) { twin[i] = static_cast<int>(j); break; }
        }
        if (twin[i] < 0) firsts.push_back(i);
    }
    std::vector<TopoDS_Face> measured;
    std::vector<size_t> at(faces.size(),0);
    for (size_t i=0;i<faces.size();++i) if (twin[i] < 0) { at[i] = measured.size(); measured.push_back(faces[i]); }
    const gp_Pnt origin = axis.Location();
    std::vector<double> mass(measured.size(),0.);
    OSD_Parallel::For(0,static_cast<int>(measured.size()),[&](int i) {
        mass[static_cast<size_t>(i)] = flux({measured[static_cast<size_t>(i)]},origin);
    });
    if (std::getenv("SOLVENT_STEP_DEBUG")) fprintf(stderr,"step: %zu of %zu faces measured, the rest turns of them\n",measured.size(),faces.size());
    double total = 0;
    for (size_t i=0;i<faces.size();++i) total += mass[at[twin[i] < 0 ? i : static_cast<size_t>(twin[i])]];
    return total;
}

// A STEP model's text, its entities formatted side by side: each thread formats a run of them in a
// writer of its own over the one model (formatting an entity reads the model and writes only its
// own lines), and the runs are joined in order between the header and the end, as the kernel's
// writer lays them out one after another. `SOLVENT_STEP_SERIAL` has the kernel's writer write it
// whole; `SOLVENT_STEP_TEXT_CHECK` has it write it as well and requires the same text.
static std::string step_text(STEPControl_Writer& writer) {
    const auto serial = [&] {
        std::ostringstream out;
        if (writer.WriteStream(out) != IFSelect_RetDone) throw std::runtime_error("STEP write failed");
        return std::move(out).str();
    };
    if (std::getenv("SOLVENT_STEP_SERIAL")) return serial();
    const Handle(StepData_StepModel) model = writer.Model();
    const Handle(StepData_Protocol) protocol = Handle(StepData_Protocol)::DownCast(writer.WS()->Protocol());
    if (model.IsNull() || protocol.IsNull()) return serial();
    std::ostringstream out;
    // (the header alone leaves out the exchange structure's first line)
    out << "ISO-10303-21;\n";
    {
        StepData_StepWriter head(model);
        head.SendModel(protocol,true);
        head.SendData();
        head.Print(out);
    }
    const int count = model->NbEntities();
    const int runs = std::max(1,std::min(count,static_cast<int>(std::max(1u,std::thread::hardware_concurrency()))*4));
    std::vector<std::string> texts(static_cast<size_t>(runs));
    OSD_Parallel::For(0,runs,[&](int r) {
        const int from = 1+static_cast<int>(static_cast<long long>(count)*r/runs),to = static_cast<int>(static_cast<long long>(count)*(r+1)/runs);
        StepData_StepWriter run(model);
        const StepData_WriterLib lib(protocol);
        for (int i=from;i<=to;++i) run.SendEntity(i,lib);
        std::ostringstream text;
        run.Print(text);
        texts[static_cast<size_t>(r)] = std::move(text).str();
    });
    for (const auto& text: texts) out << text;
    {
        StepData_StepWriter tail(model);
        tail.EndSec();
        tail.EndFile();
        tail.Print(out);
    }
    std::string text = std::move(out).str();
    if (std::getenv("SOLVENT_STEP_TEXT_CHECK")) {
        const std::string whole = serial();
        if (whole != text) {
            size_t k = 0;
            while (k < whole.size() && k < text.size() && whole[k] == text[k]) ++k;
            fprintf(stderr,"step: the text formatted side by side differs from the kernel's at byte %zu of %zu/%zu: [%s] against [%s]\n",
                k,text.size(),whole.size(),text.substr(k>40?k-40:0,120).c_str(),whole.substr(k>40?k-40:0,120).c_str());
            throw std::runtime_error("the STEP text formatted side by side differs from the kernel's");
        }
    }
    return text;
}

extern "C" {
Cad* solvent_cad_new() noexcept { try { return new Cad; } catch (...) { return nullptr; } }
void solvent_cad_free(Cad* cad) noexcept { delete cad; }
const char* solvent_cad_error(Cad*) noexcept { return last_error().c_str(); }

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
        const double volume = validate(shape);
        return cad->put(shape,true,shape.ShapeType() == TopAbs_SOLID ? volume : std::nan(""));
    });
}
int solvent_cad_revolve(Cad* cad,int face,const double* origin,const double* axis,double angle) noexcept {
    return guarded(cad,[&] {
        auto dir = direction(axis);
        if (angle < 0) dir.Reverse();
        BRepPrimAPI_MakeRevol body(cad->at(face),gp_Ax1(point(origin),dir),std::abs(angle),true);
        auto shape = body.Shape();
        const double volume = validate(shape);
        return cad->put(shape,true,shape.ShapeType() == TopAbs_SOLID ? volume : std::nan(""));
    });
}
// Booleans run with a 1e-5 mm fuzzy tolerance: near-coincident intersections
// (a wide tip cone against a blank sphere) otherwise leave an open shell.
// `operation`: 0 fuses, 1 cuts, 2 keeps what the two share.
int solvent_cad_boolean(Cad* cad,int a,int b,int operation_kind) noexcept {
    return guarded(cad,[&] {
        TopTools_ListOfShape objects,tools;
        objects.Append(cad->at(a)); tools.Append(cad->at(b));
        TopoDS_Shape result;
        if (operation_kind == 1) {
            BRepAlgoAPI_Cut operation;
            operation.SetArguments(objects); operation.SetTools(tools);
            operation.SetFuzzyValue(1e-5); operation.SetRunParallel(parallel_booleans()); operation.SetUseOBB(true);
            operation.Build();
            if (!operation.IsDone()) throw std::runtime_error("Boolean cut failed");
            result = operation.Shape();
        } else if (operation_kind == 2) {
            BRepAlgoAPI_Common operation;
            operation.SetArguments(objects); operation.SetTools(tools);
            operation.SetFuzzyValue(1e-5); operation.SetRunParallel(parallel_booleans()); operation.SetUseOBB(true);
            operation.Build();
            if (!operation.IsDone()) throw std::runtime_error("Boolean intersection failed");
            result = operation.Shape();
        } else {
            BRepAlgoAPI_Fuse operation;
            operation.SetArguments(objects); operation.SetTools(tools);
            operation.SetFuzzyValue(1e-5); operation.SetRunParallel(parallel_booleans()); operation.SetUseOBB(true);
            operation.Build();
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
        const double volume = validate(result);
        return cad->put(result,true,result.ShapeType() == TopAbs_SOLID ? volume : std::nan(""));
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
    return guarded(cad,[&] { cad->validated(id); return 0; });
}
// What a STEP file written from a solid must say of it (`solvent_cad_brep_summary`): its counts
// of solids, shells, faces, edges and vertices (those the writer writes), and each face in the order the solid's explorer lists them — the order the writer's
// closed shell lists them in — with its orientation in the solid (1 reversed, and 2 more where its
// elementary surface's placement is left-handed) and its surface: the kind
// (`GeomAbs_SurfaceType`'s number), and for an elementary surface its placement (location, axis,
// reference direction) and its radii and angle, for a B-spline its degrees, knots,
// multiplicities, poles and weights (a periodic one as the writer makes it, not periodic).
static std::vector<double> brep_summary(const TopoDS_Shape& shape) {
    std::vector<double> out = {1};
    TopTools_IndexedMapOfShape solids,shells,faces,edges;
    TopExp::MapShapes(shape,TopAbs_SOLID,solids);
    TopExp::MapShapes(shape,TopAbs_SHELL,shells);
    TopExp::MapShapes(shape,TopAbs_FACE,faces);
    TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    // The edges the writer writes: not a degenerate one, nor the seam of a face bounded by nothing
    // else but degenerate edges (a whole sphere), which it writes as one vertex loop.
    TopTools_IndexedMapOfShape unwritten;
    for (int i=1;i<=faces.Extent();++i) {
        const TopoDS_Face face = TopoDS::Face(faces(i));
        bool degenerate = false,other = false;
        for (TopExp_Explorer e(face,TopAbs_EDGE); e.More(); e.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(e.Current());
            if (BRep_Tool::Degenerated(edge)) degenerate = true;
            else if (!BRep_Tool::IsClosed(edge,face)) other = true;
        }
        if (degenerate && !other) for (TopExp_Explorer e(face,TopAbs_EDGE); e.More(); e.Next()) unwritten.Add(e.Current());
    }
    // The vertices it writes: the written edges' ends, and one vertex for each face it writes as a
    // vertex loop and nothing else.
    int solid_edges = 0,natural = 0;
    TopTools_IndexedMapOfShape ends;
    for (int i=1;i<=edges.Extent();++i)
        if (!BRep_Tool::Degenerated(TopoDS::Edge(edges(i))) && !unwritten.Contains(edges(i))) {
            ++solid_edges;
            TopExp::MapShapes(edges(i),TopAbs_VERTEX,ends);
        }
    for (int i=1;i<=faces.Extent();++i) {
        bool written = false,loop = false;
        for (TopExp_Explorer e(faces(i),TopAbs_EDGE); e.More(); e.Next()) (unwritten.Contains(e.Current()) ? loop : written) = true;
        if (loop && !written) ++natural;
    }
    for (const double n: {solids.Extent(),shells.Extent(),faces.Extent(),solid_edges,ends.Extent()+natural}) out.push_back(n);
    // a placement as the writer writes it: its location, axis and reference direction, either
    // direction reversed where the writer reverses it (`axis`, `reference`)
    const auto place = [&](const gp_Ax3& a,bool axis = false,bool reference = false) {
        const gp_XYZ z = axis ? a.Direction().XYZ().Reversed() : a.Direction().XYZ();
        const gp_XYZ x = reference ? a.XDirection().XYZ().Reversed() : a.XDirection().XYZ();
        for (const gp_XYZ& v: {a.Location().XYZ(),z,x}) { out.push_back(v.X()); out.push_back(v.Y()); out.push_back(v.Z()); }
    };
    for (TopExp_Explorer it(shape,TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        while (true) {
            const auto trimmed = Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);
            if (trimmed.IsNull()) break;
            surface = trimmed->BasisSurface();
        }
        const int kind = GeomAdaptor_Surface(surface).GetType();
        out.push_back(kind);
        // (a surface on a left-handed placement is written on a right-handed one, its first parameter
        // reversed, and the face's sense with it)
        const auto elementary = Handle(Geom_ElementarySurface)::DownCast(surface);
        const bool indirect = !elementary.IsNull() && !elementary->Position().Direct();
        out.push_back((face.Orientation() == TopAbs_REVERSED ? 1 : 0)+(indirect ? 2 : 0));
        const size_t count = out.size();
        out.push_back(0);
        // (a plane on a left-handed placement is written on its reference direction reversed)
        if (const auto s = Handle(Geom_Plane)::DownCast(surface); !s.IsNull()) place(s->Position(),false,indirect);
        else if (const auto s = Handle(Geom_CylindricalSurface)::DownCast(surface); !s.IsNull()) { place(s->Position()); out.push_back(s->Radius()); }
        else if (const auto s = Handle(Geom_ConicalSurface)::DownCast(surface); !s.IsNull()) {
            // (a cone opening against its axis is written about the axis reversed, its angle positive:
            // the same points, both parameters reversed, so the face's sense is kept)
            place(s->Position(),s->SemiAngle() < 0); out.push_back(s->RefRadius()); out.push_back(std::abs(s->SemiAngle()));
        }
        else if (const auto s = Handle(Geom_SphericalSurface)::DownCast(surface); !s.IsNull()) { place(s->Position()); out.push_back(s->Radius()); }
        else if (const auto s = Handle(Geom_ToroidalSurface)::DownCast(surface); !s.IsNull()) {
            place(s->Position()); out.push_back(s->MajorRadius()); out.push_back(s->MinorRadius());
        }
        else if (auto s = Handle(Geom_BSplineSurface)::DownCast(surface); !s.IsNull()) {
            if (s->IsUPeriodic() || s->IsVPeriodic()) {
                s = Handle(Geom_BSplineSurface)::DownCast(s->Copy());
                if (s->IsUPeriodic()) s->SetUNotPeriodic();
                if (s->IsVPeriodic()) s->SetVNotPeriodic();
            }
            const bool rational = s->IsURational() || s->IsVRational();
            for (const double n: {s->UDegree(),s->VDegree(),s->NbUPoles(),s->NbVPoles(),s->NbUKnots(),s->NbVKnots()}) out.push_back(n);
            out.push_back(rational ? 1 : 0);
            for (int k=1;k<=s->NbUKnots();++k) out.push_back(s->UKnot(k));
            for (int k=1;k<=s->NbUKnots();++k) out.push_back(s->UMultiplicity(k));
            for (int k=1;k<=s->NbVKnots();++k) out.push_back(s->VKnot(k));
            for (int k=1;k<=s->NbVKnots();++k) out.push_back(s->VMultiplicity(k));
            for (int i=1;i<=s->NbUPoles();++i) for (int j=1;j<=s->NbVPoles();++j) {
                const gp_Pnt p = s->Pole(i,j); out.push_back(p.X()); out.push_back(p.Y()); out.push_back(p.Z());
            }
            if (rational) for (int i=1;i<=s->NbUPoles();++i) for (int j=1;j<=s->NbVPoles();++j) out.push_back(s->Weight(i,j));
        }
        out[count] = static_cast<double>(out.size()-count-1);
    }
    return out;
}

// A STEP file of a stored solid: transferred, its text formatted (`step_text`) and written, and —
// `full` — read back as a consumer's reader takes it (its default repairs), checked and measured
// against the solid. Without `full` the caller verifies the file against `solvent_cad_brep_summary`.
// `unchecked`: the caller checks the solid itself beside the writing (a pattern's union,
// `solvent_cad_pattern_check`), and it is not validated here first.
int solvent_cad_step(Cad* cad,int id,const char* path,int full,int unchecked) noexcept {
    return guarded(cad,[&] {
        // The CLI reserves stdout for its JSON/text report. Restore the stream
        // even if STEP construction throws; this host runs synchronously.
        struct LogStream {
            std::streambuf* old = std::cout.rdbuf(std::cerr.rdbuf());
            ~LogStream() { std::cout.rdbuf(old); }
        } log_stream;
        if (!unchecked || full) cad->validated(id);
        const TopoDS_Shape shape = cad->at(id);
        const bool debug = std::getenv("SOLVENT_STEP_DEBUG") != nullptr;
        auto clock = std::chrono::steady_clock::now();
        const auto lap = [&](const char* step) {
            const auto now = std::chrono::steady_clock::now();
            if (debug) fprintf(stderr,"step: %s %.2f s\n",step,std::chrono::duration<double>(now-clock).count());
            clock = now;
        };
        STEPControl_Writer writer;
        if (writer.Transfer(shape,STEPControl_AsIs) != IFSelect_RetDone) throw std::runtime_error("STEP write failed");
        lap("transferred");
        // Written into memory and then to the file in one piece (the writer's file stream flushes a
        // line at a time), and read back from the same bytes.
        const std::string text = step_text(writer);
        {
            FILE* file = std::fopen(path,"wb");
            if (!file) throw std::runtime_error(std::string("cannot write STEP file ")+path);
            const size_t wrote = std::fwrite(text.data(),1,text.size(),file);
            if (std::fclose(file) != 0 || wrote != text.size()) throw std::runtime_error(std::string("STEP write to ")+path+" failed");
        }
        lap("written");
        if (!full) return 0;
        // Read back from the same bytes, as a consumer's reader takes them (its default repairs).
        STEPControl_Reader reader;
        {
            std::istringstream in(text);
            if (reader.ReadStream(path,in) != IFSelect_RetDone) throw std::runtime_error("STEP reimport failed");
        }
        lap("read");
        if (!reader.TransferRoots()) throw std::runtime_error("STEP reimport failed");
        lap("transferred back");
        auto imported = reader.OneShape();
        const double before = cad->volume_of(id);
        // The reading checked and oriented as any solid is, and measured (a pattern's as one,
        // `patterned_volume`; `SOLVENT_STEP_CHECK` measures every face of it).
        if (!valid_solid(imported)) throw std::runtime_error("native solid is invalid:"+invalidity(imported));
        lap("checked");
        TopoDS_Shape oriented = imported;
        if (oriented.ShapeType() == TopAbs_SOLID) {
            auto solid = TopoDS::Solid(oriented);
            if (!BRepLib::OrientClosedSolid(solid)) throw std::runtime_error("solid is open");
            oriented = solid;
        }
        // A pattern's reading measured as one: its sheets' copies as turns of one.
        gp_Ax1 axis;
        int copies = 0;
        double after = std::nan("");
        if (cad->pattern(id,axis,copies) && std::getenv("SOLVENT_STEP_CHECK") == nullptr && oriented.ShapeType() == TopAbs_SOLID) {
            after = patterned_volume(oriented,axis,copies,1e-8);
            if (!(after > 0)) throw std::runtime_error("solid has no positive volume");
            if (debug) fprintf(stderr,"step: the reading measures %.12g as copies, %.12g whole; %.12g written\n",after,volume(oriented),before);
        } else after = validate(imported,nullptr,true);
        lap("measured");
        if (std::abs(before-after) <= 1e-9+1e-7*std::abs(before)) return 0;
        // A reader may move the boundary within the tolerance the shape itself
        // carries, so the admissible volume change is that slack over the whole
        // surface; an analytic solid at kernel precision keeps the strict ratio.
        const double slack = BRep_Tool::MaxTolerance(shape,TopAbs_VERTEX)*area(shape);
        lap("area");
        if (std::abs(before-after) > std::max(1e-9+1e-7*std::abs(before),slack))
            throw std::runtime_error("STEP round trip changed solid volume from "
                +std::to_string(before)+" to "+std::to_string(after));
        return 0;
    });
}
// A stored shape's `brep_summary`, `capacity` doubles of it written to `output`: its length.
int solvent_cad_brep_summary(Cad* cad,int id,double* output,int capacity) noexcept {
    return guarded(cad,[&] {
        const std::vector<double> summary = brep_summary(cad->at(id));
        if (output) std::copy_n(summary.begin(),std::min(summary.size(),static_cast<size_t>(std::max(capacity,0))),output);
        return static_cast<int>(summary.size());
    });
}
// Read a STEP file's shape into the session, for a meter measuring what was written; nothing
// is validated or repaired, since the file is what is being judged.
int solvent_cad_read_step(Cad* cad,const char* path) noexcept {
    return guarded(cad,[&] {
        struct LogStream {
            std::streambuf* old = std::cout.rdbuf(std::cerr.rdbuf());
            ~LogStream() { std::cout.rdbuf(old); }
        } log_stream;
        if (!path) throw std::runtime_error("STEP read needs a path");
        STEPControl_Reader reader;
        if (reader.ReadFile(path) != IFSelect_RetDone || !reader.TransferRoots())
            throw std::runtime_error(std::string("cannot read STEP file ")+path);
        auto shape = reader.OneShape();
        if (shape.IsNull()) throw std::runtime_error("STEP file holds no shape");
        return cad->put(shape);
    });
}
// Mesh a shape afresh at an absolute `deflection` (mm) and an `angular` one (radians), dropping
// any triangulation it had, so a finer mesh replaces a coarser one.
int solvent_cad_remesh(Cad* cad,int id,double deflection,double angular) noexcept {
    return guarded(cad,[&] {
        if (!std::isfinite(deflection) || deflection <= 0 || !std::isfinite(angular) || angular <= 0)
            throw std::runtime_error("meshing needs a positive deflection and angle");
        cad->validated(id);
        auto& shape = cad->at(id);
        BRepTools::Clean(shape);
        BRepMesh_IncrementalMesh mesher(shape,deflection,false,angular,false);
        if (!mesher.IsDone()) throw std::runtime_error("native tessellation failed");
        return 0;
    });
}
// The sag of every face of a meshed shape (`face_sag`). `output`: the distance (mm) and where.
int solvent_cad_mesh_sag(Cad* cad,int id,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("mesh sag needs an output buffer");
        double worst = 0; gp_Pnt at;
        for (TopExp_Explorer it(cad->at(id),TopAbs_FACE); it.More(); it.Next()) face_sag(TopoDS::Face(it.Current()),worst,at);
        output[0] = worst; output[1] = at.X(); output[2] = at.Y(); output[3] = at.Z();
        return 0;
    });
}
// `deflection` is absolute millimetres, matching the native construction recipe, and `angular`
// radians. These are tessellator controls, not an end-to-end geometry error certificate.
int solvent_cad_stl(Cad* cad,int id,const char* path,double deflection,double angular) noexcept {
    return guarded(cad,[&] {
        if (!std::isfinite(deflection) || deflection <= 0 || !std::isfinite(angular) || angular <= 0)
            throw std::runtime_error("STL meshing needs a positive deflection and angle");
        cad->validated(id);
        auto& shape = cad->at(id);
        BRepMesh_IncrementalMesh mesher(shape,deflection,false,angular,false);
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
