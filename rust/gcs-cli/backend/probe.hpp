#pragma once
// Phase 0's measurement (docs/rust-kernel-plan.md): with `SOLVENT_ABI_TRACE` naming a file, every
// entry point's time, every Boolean's face pairs that actually intersect — their surface kinds,
// each intersection curve's length and the least angle between the surfaces along it — and the
// exported shape's smallest face and edge, one tab-separated line each, on one steady clock the
// Rust side marks its stages on too (`solvent_cad_probe_mark`). Off, it costs a pointer test.
//
//   call    NAME    START   SECONDS THREAD
//   mark    STAGE   AT      THREAD
//   ff      WHAT    KIND1   KIND2   LENGTH  MIN_ANGLE_DEG   TANGENT POINTS
//   boolean WHAT    PAIRS   CURVES
//   shape   WHAT    FACES   EDGES   MIN_FACE_AREA   MIN_EDGE_LENGTH KIND:COUNT...
//
// Kinds are `GeomAbs_SurfaceType`'s numbers (0 plane … 6 B-spline, 7 revolution, 8 extrusion).
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BOPAlgo_PaveFiller.hxx>
#include <BOPDS_DS.hxx>
#include <BOPDS_Interf.hxx>
#include <BOPDS_Curve.hxx>
#include <IntTools_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>

namespace probe {

inline const char* path() { static const char* p = std::getenv("SOLVENT_ABI_TRACE"); return p; }
inline bool on() { return path() != nullptr; }

/// Seconds on the one clock every line is written against.
inline double now() {
    using Clock = std::chrono::steady_clock;
    static const Clock::time_point start = Clock::now();
    return std::chrono::duration<double>(Clock::now()-start).count();
}

/// A small number for the calling thread, in the order threads first write.
inline int thread() {
    static std::atomic<int> next{0};
    thread_local const int id = next++;
    return id;
}

inline void write(const std::string& line) {
    static std::mutex lock;
    static FILE* file = nullptr;
    std::lock_guard<std::mutex> hold(lock);
    if (!file) file = std::fopen(path(),"a");
    if (!file) return;
    std::fputs(line.c_str(),file);
    std::fputc('\n',file);
    std::fflush(file);
}

/// An entry point's time, from construction to the end of its scope.
struct Scope {
    const char* name;
    double start = 0;
    bool active;
    explicit Scope(const char* n): name(n),active(on()) { if (active) start = now(); }
    ~Scope() {
        if (!active) return;
        char line[256];
        std::snprintf(line,sizeof line,"call\t%s\t%.6f\t%.6f\t%d",name,start,now()-start,thread());
        write(line);
    }
};

/// The least angle (degrees, 0 to 90) between two faces' surfaces along a curve, sampled.
inline double least_angle(const Handle(Geom_Curve)& curve,double t0,double t1,const TopoDS_Face& a,const TopoDS_Face& b) {
    const Handle(Geom_Surface) sa = BRep_Tool::Surface(a), sb = BRep_Tool::Surface(b);
    double least = 90.;
    for (int k=0;k<=16;++k) {
        const gp_Pnt p = curve->Value(t0+(t1-t0)*k/16.);
        GeomAPI_ProjectPointOnSurf pa(p,sa), pb(p,sb);
        if (!pa.NbPoints() || !pb.NbPoints()) continue;
        double ua,va,ub,vb;
        pa.LowerDistanceParameters(ua,va); pb.LowerDistanceParameters(ub,vb);
        GeomLProp_SLProps la(sa,ua,va,1,1e-9), lb(sb,ub,vb,1,1e-9);
        if (!la.IsNormalDefined() || !lb.IsNormalDefined()) continue;
        const double c = std::min(1.,std::fabs(la.Normal().Dot(lb.Normal())));
        least = std::min(least,std::acos(c)*180./M_PI);
    }
    return least;
}

/// Every face pair a Boolean found meeting, and what it found there.
inline void boolean(const char* what,const BRepAlgoAPI_BuilderAlgo& algorithm) {
    if (!on()) return;
    const BOPAlgo_PPaveFiller& filler = algorithm.DSFiller();
    if (!filler) return;
    const BOPDS_PDS ds = filler->PDS();
    if (!ds) return;
    BOPDS_VectorOfInterfFF& pairs = ds->InterfFF();
    int curves = 0;
    for (int i=0;i<pairs.Length();++i) {
        const BOPDS_InterfFF& pair = pairs(i);
        int n1,n2;
        pair.Indices(n1,n2);
        const TopoDS_Face& a = TopoDS::Face(ds->Shape(n1));
        const TopoDS_Face& b = TopoDS::Face(ds->Shape(n2));
        const int k1 = BRepAdaptor_Surface(a,false).GetType(), k2 = BRepAdaptor_Surface(b,false).GetType();
        const BOPDS_VectorOfCurve& found = pair.Curves();
        char line[512];
        if (found.Length() == 0) {
            std::snprintf(line,sizeof line,"ff\t%s\t%d\t%d\t0\tnan\t%d\t%d",what,k1,k2,int(pair.TangentFaces()),pair.Points().Length());
            write(line);
            continue;
        }
        for (int c=0;c<found.Length();++c) {
            const IntTools_Curve& curve = found(c).Curve();
            const Handle(Geom_Curve)& geometry = curve.Curve();
            double t0,t1;
            gp_Pnt p0,p1;
            if (geometry.IsNull() || !curve.Bounds(t0,t1,p0,p1)) continue;
            ++curves;
            double length = 0;
            try { length = GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(geometry,t0,t1)); } catch (...) {}
            std::snprintf(line,sizeof line,"ff\t%s\t%d\t%d\t%.9g\t%.6g\t%d\t%d",what,k1,k2,length,
                least_angle(geometry,t0,t1,a,b),int(pair.TangentFaces()),pair.Points().Length());
            write(line);
        }
    }
    char line[256];
    std::snprintf(line,sizeof line,"boolean\t%s\t%d\t%d",what,pairs.Length(),curves);
    write(line);
}

/// A shape's faces by kind, and its smallest face and shortest edge.
inline void shape(const char* what,const TopoDS_Shape& s) {
    if (!on()) return;
    std::map<int,int> kinds;
    int faces = 0,edges = 0;
    double least_area = INFINITY,least_edge = INFINITY;
    for (TopExp_Explorer it(s,TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face& face = TopoDS::Face(it.Current());
        ++faces;
        ++kinds[BRepAdaptor_Surface(face,false).GetType()];
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face,props);
        least_area = std::min(least_area,props.Mass());
    }
    TopTools_IndexedMapOfShape all;
    TopExp::MapShapes(s,TopAbs_EDGE,all);
    for (int i=1;i<=all.Extent();++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(all(i));
        if (BRep_Tool::Degenerated(edge)) continue;
        ++edges;
        GProp_GProps props;
        BRepGProp::LinearProperties(edge,props);
        least_edge = std::min(least_edge,props.Mass());
    }
    std::string line = std::string("shape\t")+what+"\t"+std::to_string(faces)+"\t"+std::to_string(edges);
    char numbers[96];
    std::snprintf(numbers,sizeof numbers,"\t%.9g\t%.9g",least_area,least_edge);
    line += numbers;
    for (auto [k,n]: kinds) line += "\t"+std::to_string(k)+":"+std::to_string(n);
    write(line);
}

}

#define SOLVENT_PROBE(name) probe::Scope solvent_probe_scope_(name)
