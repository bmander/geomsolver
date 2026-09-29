// Cell decomposition of a stock solid by candidate boundary sheets.
// The kernel arranges; material selection is the caller's, from the source field.
#include "occt.hpp"
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BOPAlgo_GlueEnum.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom2d_Curve.hxx>
#include <gp_Vec2d.hxx>
#include <gp_Lin.hxx>
#include <ElCLib.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <Precision.hxx>
#include <gp_Trsf.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <gp_Lin.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepGProp.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>
#include <sstream>

// Adaptive quadrature: the default fixed rule is inaccurate on spline faces.
static double volume(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape,props,1e-9,false,false);
    return props.Mass();
}

// A kernel operation's time budget: the algorithm polls UserBreak and stops once it is past.
// A split that grinds on near-tangent sheets is refused by name, not waited on. The budget is
// the process's processor time, not the clock's: a machine busy with other work (a test suite
// run in parallel) slows a split without its grinding.
static double processor_seconds() { return static_cast<double>(std::clock())/CLOCKS_PER_SEC; }
class Deadline: public Message_ProgressIndicator {
    double end;
public:
    explicit Deadline(double seconds): end(processor_seconds()+seconds) {}
    Standard_Boolean UserBreak() override { return processor_seconds() > end; }
    void Show(const Message_ProgressScope&,const Standard_Boolean) override {}
};

static void check_algorithm(BRepAlgoAPI_BuilderAlgo& algorithm,const char* what) {
    if (!algorithm.IsDone() || algorithm.HasErrors()) {
        std::ostringstream message;
        message << what << " failed: ";
        algorithm.DumpErrors(message); algorithm.DumpWarnings(message);
        throw std::runtime_error(message.str());
    }
}

// A union made valid, its faces on one support merged where that costs nothing, and stored as
// one closed solid with its volume. Unification is a simplification, not a requirement: keep the
// union when merging faces on one support produces an invalid shape or has to widen a tolerance
// to do so (it once left a 40 mm vertex tolerance). Unify a copy: the algorithm updates
// tolerances on vertices it shares with its input, which would silently widen the union kept.
static int united(Cad* cad,TopoDS_Shape result,const char* what) {
    const bool debug = std::getenv("SOLVENT_SECTOR_DEBUG") != nullptr;
    double clock = processor_seconds();
    const auto lap = [&](const char* step) {
        if (debug) fprintf(stderr,"sector: %s: %s %.2f s\n",what,step,processor_seconds()-clock);
        clock = processor_seconds();
    };
    if (!BRepCheck_Analyzer(result).IsValid()) throw std::runtime_error(std::string(what)+" is invalid before unification");
    lap("checked");
    const double before = BRep_Tool::MaxTolerance(result,TopAbs_VERTEX);
    ShapeUpgrade_UnifySameDomain unify(BRepBuilderAPI_Copy(result).Shape(),true,true,false);
    unify.Build();
    TopoDS_Shape unified = unify.Shape();
    lap("unified");
    if (debug) {
        int a = 0,b = 0;
        for (TopExp_Explorer it(result,TopAbs_FACE); it.More(); it.Next()) ++a;
        if (!unified.IsNull()) for (TopExp_Explorer it(unified,TopAbs_FACE); it.More(); it.Next()) ++b;
        fprintf(stderr,"sector: %s: %d faces, %d unified; vertex tolerance %.3g, %.3g unified; unified valid %d\n",what,a,b,before,
            unified.IsNull() ? -1. : BRep_Tool::MaxTolerance(unified,TopAbs_VERTEX),unified.IsNull() ? -1 : int(BRepCheck_Analyzer(unified).IsValid()));
    }
    if (!unified.IsNull() && BRepCheck_Analyzer(unified).IsValid()
        && BRep_Tool::MaxTolerance(unified,TopAbs_VERTEX) <= before*1.001+1e-9) result = unified;
    lap("checked the unified");
    int solids = 0;
    for (TopExp_Explorer it(result,TopAbs_SOLID); it.More(); it.Next()) { ++solids; result = it.Current(); }
    if (solids != 1) throw std::runtime_error(std::string(what)+" is not one connected solid");
    const double v = validate(result);
    lap("validated and measured");
    const int id = cad->put(result);
    cad->valid[static_cast<size_t>(id)] = 1;
    cad->volumes[static_cast<size_t>(id)] = v;
    return id;
}

extern "C" {
int solvent_cad_split_solid_fuzzy(Cad* cad,int solid,const int* tools,int count,double fuzzy) noexcept;
// Partition one solid by faces or solids. Every cell is retained as a separate
// solid; nothing here decides which cells are material. Sheets that end inside
// the stock leave cells unseparated, which the caller's classification must
// detect as a cell whose samples disagree.
// The default fuzzy tolerance of 1e-5 mm resolves near-coincident intersections
// (a gear tip cone against its blank sphere) at a stated cost far below the
// export accuracy target; it is not a substitute for exact geometry.
int solvent_cad_split_solid(Cad* cad,int solid,const int* tools,int count) noexcept {
    return solvent_cad_split_solid_fuzzy(cad,solid,tools,count,1e-5);
}
// The same split with an explicit fuzzy tolerance.
int solvent_cad_split_solid_fuzzy(Cad* cad,int solid,const int* tools,int count,double fuzzy) noexcept {
    return guarded(cad,[&] {
        if (!tools || count < 1 || count > 4096)
            throw std::runtime_error("solid split requires 1..4096 tools");
        auto& stock = cad->at(solid);
        if (stock.IsNull() || !BRepCheck_Analyzer(stock).IsValid())
            throw std::runtime_error("invalid stock solid");
        TopTools_ListOfShape objects,cutters;
        objects.Append(stock);
        for (int i=0;i<count;++i) {
            const auto& tool = cad->at(tools[i]);
            if (tool.IsNull() || !BRepCheck_Analyzer(tool).IsValid())
                throw std::runtime_error("invalid split tool");
            cutters.Append(tool);
        }
        BRepAlgoAPI_Splitter split;
        split.SetArguments(objects); split.SetTools(cutters);
        // Oriented boxes cull face pairs the axis-aligned ones cannot (the tilted sheets):
        // a quarter of the split, and the same cells.
        split.SetNonDestructive(true); split.SetRunParallel(false); split.SetUseOBB(Standard_True);
        if (fuzzy > 0) split.SetFuzzyValue(fuzzy);
        // The kernel polls the break only between its phases, so a stop can come well after the
        // budget; the message says both.
        const double budget = 15.+5.*count;
        const double started = processor_seconds();
        Handle(Deadline) deadline = new Deadline(budget);
        split.Build(deadline->Start());
        if (deadline->UserBreak()) {
            const double spent = processor_seconds()-started;
            throw std::runtime_error("the split did not finish within its "+std::to_string(int(budget))
                +" s processor budget (stopped after "+std::to_string(int(spent))+" s)");
        }
        check_algorithm(split,"solid split");
        const auto result = split.Shape();
        int cells = 0;
        for (TopExp_Explorer it(result,TopAbs_SOLID); it.More(); it.Next()) ++cells;
        if (!cells) throw std::runtime_error("solid split produced no cells");
        TopoDS_Shape checked = result;
        // Every cell's volume is measured here; the cells, listed next, keep it.
        validate(checked,&cad->measured);
        return cad->put(result);
    });
}

// Count first with output=null/capacity=0, then retrieve session-owned handles.
int solvent_cad_solids(Cad* cad,int source,int* output,int capacity) noexcept {
    return guarded(cad,[&] {
        TopTools_IndexedMapOfShape solids;
        TopExp::MapShapes(cad->at(source),TopAbs_SOLID,solids);
        const int count = solids.Extent();
        if (!output && capacity == 0) return count;
        if (!output || capacity < count) throw std::runtime_error("solid buffer is too small");
        for (int i=1;i<=count;++i) {
            output[i-1] = cad->put(solids(i));
            if (cad->measured.IsBound(solids(i)))
                cad->volumes[static_cast<size_t>(output[i-1])] = cad->measured.Find(solids(i));
        }
        return count;
    });
}

// Interior points of a cell with their true distance to its boundary, best
// first. The classifier's tolerance test only looks along its ray, so it is not
// a distance; the extrema solver against the shell is. Candidates are stepped in
// from the cell's faces, then taken halfway along rays between its faces, then
// from a bounded grid over the box, classified at a small tolerance, and the
// distance is measured for at most `measure` spread candidates. No centroid: integrating a spline cell for it cost seconds, and the
// cell's volume is measured once already where the partition is listed. Each
// output row is x, y, z, distance.
int solvent_cad_solid_samples(Cad* cad,int id,double* output,int capacity,int measure) noexcept {
    return guarded(cad,[&] {
        if (!output || capacity < 1 || measure < 1)
            throw std::runtime_error("solid samples need a buffer and positive counts");
        const auto& shape = cad->at(id);
        if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID)
            throw std::runtime_error("solid samples need one solid");
        Bnd_Box box;
        BRepBndLib::Add(shape,box,false);
        double x0,y0,z0,x1,y1,z1;
        box.Get(x0,y0,z0,x1,y1,z1);
        BRepClass3d_SolidClassifier classify(shape);
        std::vector<gp_Pnt> inside;
        const auto consider = [&](const gp_Pnt& p) {
            if (static_cast<int>(inside.size()) >= measure) return;
            classify.Perform(p,Precision::Confusion());
            if (classify.State() == TopAbs_IN) inside.push_back(p);
        };
        const double diagonal = std::hypot(std::hypot(x1-x0,y1-y0),z1-z0);
        // A face's point at (u, v) and its outward unit normal there; false where the surface is
        // singular.
        const auto outward = [](const TopoDS_Face& face,const BRepAdaptor_Surface& surface,double u,double v,
            gp_Pnt& p,gp_Vec& n) {
            gp_Vec du,dv;
            surface.D1(u,v,p,du,dv);
            n = du.Crossed(dv);
            if (n.Magnitude() <= 1e-12) return false;
            n.Normalize();
            if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
            return true;
        };
        // Where a ray from a point of `face` along its inward normal next meets the shell, if it
        // does: the points before it are inside the cell by geometry alone. The face it starts
        // on is met again within its own tolerance, which a split leaves at tens of nanometres.
        IntCurvesFace_ShapeIntersector rays;
        rays.Load(shape,Precision::Confusion());
        const auto exit = [&](const gp_Pnt& p,const gp_Vec& inward,const TopoDS_Face& face) {
            double first = Precision::Infinite();
            const double own = 10.*BRep_Tool::Tolerance(face);
            rays.Perform(gp_Lin(p,gp_Dir(inward)),diagonal*1e-7,Precision::Infinite());
            if (rays.IsDone()) for (int k=1;k<=rays.NbPnt();++k) {
                const double w = rays.WParameter(k);
                if (w <= own && rays.Face(k).IsSame(face)) continue;
                first = std::min(first,w);
            }
            return first;
        };
        // A little inward from points of each face, along its inward normal: nearly always
        // inside, where a thin cell leaves most of a grid over its box outside and each miss
        // costs a classification (32 ms each on a tooth space's spline faces). The step is a
        // fraction of the box, then a finer one. The classifier decides each candidate, and the
        // ray along the normal must not have left the cell before it: on a sliver the
        // classifier has put one point inside two cells of one partition.
        const auto from_faces = [&](const double step) {
            for (TopExp_Explorer it(shape,TopAbs_FACE); it.More() && static_cast<int>(inside.size()) < measure; it.Next()) {
                const TopoDS_Face face = TopoDS::Face(it.Current());
                double u0,u1,v0,v1;
                BRepTools::UVBounds(face,u0,u1,v0,v1);
                BRepAdaptor_Surface surface(face);
                for (const auto& [fu,fv]: {std::pair{0.5,0.5},{0.25,0.25},{0.75,0.75},{0.25,0.75},{0.75,0.25}}) {
                    gp_Pnt p; gp_Vec n;
                    if (!outward(face,surface,u0+(u1-u0)*fu,v0+(v1-v0)*fv,p,n)) continue;
                    // The ray first: on a sliver it rules out most candidates for less than
                    // a classification each.
                    if (static_cast<int>(inside.size()) >= measure || exit(p,n.Reversed(),face) <= step) continue;
                    consider(p.Translated(n.Multiplied(-step)));
                }
            }
        };
        for (const double step: {diagonal*0.02,diagonal*0.005}) from_faces(step);
        // Distance to the boundary against the whole shell at once: the extrema
        // solver culls faces by bounding box, where a face-by-face loop does not.
        TopoDS_Shape boundary;
        for (TopExp_Explorer it(shape,TopAbs_SHELL); it.More(); it.Next()) { boundary = it.Current(); break; }
        if (boundary.IsNull()) throw std::runtime_error("cell has no shell");
        std::vector<std::pair<double,gp_Pnt>> measured;
        const auto measure_all = [&] {
            for (const auto& p: inside) {
                BRepExtrema_DistShapeShape distance(BRepBuilderAPI_MakeVertex(p).Vertex(),boundary);
                if (!distance.IsDone() || distance.NbSolution() < 1) continue;
                measured.emplace_back(distance.Value(),p);
            }
            inside.clear();
        };
        // A sliver long against its thickness (a chamfer's wedge along a tooth) leaves most
        // candidates above outside, or on its boundary, and nearly all of a grid over its box:
        // from points inside each face's trimmed domain, the middle of the ray along the inward
        // normal to where it next meets the shell. The caller probes only a sample more than
        // 0.2 µm from the boundary (`classify`), and wants `capacity` of them; the grid stays
        // the last resort, since each of its misses costs a classification.
        const auto clear = [&] { return static_cast<int>(std::count_if(measured.begin(),measured.end(),
            [](const auto& m) { return m.first > 2e-4; })); };
        measure_all();
        if (clear() < capacity) {
            for (TopExp_Explorer it(shape,TopAbs_FACE); it.More() && static_cast<int>(inside.size()) < measure; it.Next()) {
                const TopoDS_Face face = TopoDS::Face(it.Current());
                double u0,u1,v0,v1;
                BRepTools::UVBounds(face,u0,u1,v0,v1);
                BRepAdaptor_Surface surface(face);
                BRepTopAdaptor_FClass2d domain(face,Precision::PConfusion());
                for (int i=0;i<5 && static_cast<int>(inside.size()) < measure;++i) for (int j=0;j<5;++j) {
                    const double u = u0+(u1-u0)*(i+0.5)/5, v = v0+(v1-v0)*(j+0.5)/5;
                    if (domain.Perform(gp_Pnt2d(u,v)) != TopAbs_IN) continue;
                    gp_Pnt p; gp_Vec n;
                    if (!outward(face,surface,u,v,p,n)) continue;
                    const double first = exit(p,n.Reversed(),face);
                    if (first >= Precision::Infinite()) continue;
                    inside.push_back(p.Translated(n.Multiplied(-0.5*first)));
                    if (static_cast<int>(inside.size()) >= measure) break;
                }
            }
            measure_all();
        }
        // A coarse grid visited in a spread order, so early candidates are far apart.
        const int n = 5;
        for (int pass=0;pass<2 && clear() < capacity;++pass) {
            for (int i=0;i<n;++i) for (int j=0;j<n;++j) for (int k=0;k<n;++k) {
                if (((i+j+k)&1) != pass) continue;
                if (static_cast<int>(inside.size()) >= measure) break;
                const gp_Pnt g(x0+(x1-x0)*(i+0.5)/n,y0+(y1-y0)*(j+0.5)/n,z0+(z1-z0)*(k+0.5)/n);
                classify.Perform(g,Precision::Confusion());
                if (classify.State() == TopAbs_IN) inside.push_back(g);
            }
            measure_all();
        }
        if (measured.empty()) return 0;
        std::sort(measured.begin(),measured.end(),[](const auto& a,const auto& b) { return a.first > b.first; });
        int written = 0;
        for (const auto& [d,p]: measured) {
            if (written >= capacity) break;
            output[4*written] = p.X(); output[4*written+1] = p.Y(); output[4*written+2] = p.Z(); output[4*written+3] = d;
            ++written;
        }
        return written;
    });
}

// Unite selected cells of one partition into a solid. Cells share exact faces
// from the split, so the union removes their common walls; faces lying on one
// supporting surface are then merged. The result must be one valid closed solid.
int solvent_cad_fuse(Cad* cad,const int* ids,int count) noexcept {
    return guarded(cad,[&] {
        if (!ids || count < 1 || count > 65536)
            throw std::runtime_error("fuse requires 1..65536 solids");
        TopoDS_Shape result;
        if (count == 1) result = cad->at(ids[0]);
        else {
            TopTools_ListOfShape objects,tools;
            objects.Append(cad->at(ids[0]));
            for (int i=1;i<count;++i) tools.Append(cad->at(ids[i]));
            BRepAlgoAPI_Fuse fuse;
            fuse.SetArguments(objects); fuse.SetTools(tools);
            fuse.SetRunParallel(false);
            fuse.Build();
            check_algorithm(fuse,"cell union");
            result = fuse.Shape();
        }
        return united(cad,result,"cell union");
    });
}

// A face of revolution about `axis`: an elementary surface (not a plane) whose own axis is that
// line, either way along it. Only these are carried onto one surface and one period.
static bool about(const TopoDS_Face& face,const gp_Ax1& axis) {
    TopLoc_Location there;
    const auto elementary = Handle(Geom_ElementarySurface)::DownCast(BRep_Tool::Surface(face,there));
    if (elementary.IsNull() || elementary->IsKind(STANDARD_TYPE(Geom_Plane)) || !there.IsIdentity()) return false;
    const gp_Ax1 own = elementary->Position().Axis();
    return own.Direction().IsParallel(axis.Direction(),1e-12)
        && gp_Lin(axis).Distance(own.Location()) <= 1e-9*(1.+own.Location().Distance(gp_Pnt(0,0,0)));
}

// Put `face` on `surface`, its pcurves (on the surface it carries now) moved along u by `shift`:
// its own surface is `surface` turned about its axis, or `surface` itself a period on.
static void reseat(const TopoDS_Face& face,const Handle(Geom_Surface)& surface,double shift) {
    BRep_Builder builder;
    const gp_Vec2d by(shift,0);
    for (TopExp_Explorer e(face,TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(e.Current());
        double f,l;
        const double tolerance = BRep_Tool::Tolerance(edge);
        if (BRep_Tool::IsClosed(edge,face)) {
            Handle(Geom2d_Curve) c1 = BRep_Tool::CurveOnSurface(TopoDS::Edge(edge.Oriented(TopAbs_FORWARD)),face,f,l);
            Handle(Geom2d_Curve) c2 = BRep_Tool::CurveOnSurface(TopoDS::Edge(edge.Oriented(TopAbs_REVERSED)),face,f,l);
            if (c1.IsNull() || c2.IsNull()) throw std::runtime_error("a seam without its pcurves");
            builder.UpdateEdge(TopoDS::Edge(edge.Oriented(TopAbs_FORWARD)),Handle(Geom2d_Curve)::DownCast(c1->Translated(by)),
                Handle(Geom2d_Curve)::DownCast(c2->Translated(by)),surface,TopLoc_Location(),tolerance);
        } else {
            Handle(Geom2d_Curve) c = BRep_Tool::CurveOnSurface(edge,face,f,l);
            if (c.IsNull()) throw std::runtime_error("an edge without its pcurve");
            builder.UpdateEdge(edge,Handle(Geom2d_Curve)::DownCast(c->Translated(by)),surface,TopLoc_Location(),tolerance);
        }
        builder.Range(edge,surface,TopLoc_Location(),f,l);
    }
    builder.UpdateFace(face,surface,TopLoc_Location(),BRep_Tool::Tolerance(face));
}

// A copy of `source` turned by `angle` about `axis`, its faces of revolution about that line put back
// on the source's own surfaces: turning a cone or a sphere about its own axis moves its parameters by
// the angle and nothing else, so the copies' pieces of one blank face then share one surface, as the
// pieces of a face a split cuts do, and a union can merge them.
static TopoDS_Shape turned_copy(const TopoDS_Shape& source,const gp_Ax1& axis,double angle) {
    gp_Trsf turn;
    turn.SetRotation(axis,angle);
    BRepBuilderAPI_Transform moved(source,turn,true);
    if (!moved.IsDone()) throw std::runtime_error("a pattern's turn failed");
    TopoDS_Shape copy = moved.Shape();
    TopExp_Explorer from(source,TopAbs_FACE),to(copy,TopAbs_FACE);
    for (; from.More() && to.More(); from.Next(),to.Next()) {
        const TopoDS_Face original = TopoDS::Face(from.Current());
        if (!about(original,axis)) continue;
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(original);
        // which way the parameter runs about the axis: the turn is a shift of u by +angle or -angle
        double u0,u1,v0,v1;
        BRepTools::UVBounds(original,u0,u1,v0,v1);
        const gp_Pnt p = surface->Value(u0,v0).Transformed(turn);
        double shift = 0;
        for (const double sign: {1.,-1.})
            if (surface->Value(u0+sign*angle,v0).Distance(p) <= 1e-9*(1.+p.Distance(gp_Pnt(0,0,0)))) { shift = sign*angle; break; }
        if (shift == 0) throw std::runtime_error("a turned face's parameters are not its surface's turned");
        reseat(TopoDS::Face(to.Current()),surface,shift);
    }
    return copy;
}

// A piece of a ring (a face of revolution the copies close round the axis) moved into u in
// [0, 2π], split along its surface's seam where it crosses it: pieces in one period meet their
// neighbours on pcurves that agree, and the ring they make closes on the seam line, an iso line,
// as the blank's own face did. (Closed on the junction between two copies instead, the ring's
// parameters would span more than a period, which the mesher does not read.)
static std::vector<TopoDS_Face> into_one_period(const TopoDS_Face& face,const gp_Ax1& axis,double reach) {
    const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    const double period = 2*M_PI,slack = 1e-9;
    double u0,u1,v0,v1;
    BRepTools::UVBounds(face,u0,u1,v0,v1);
    const double m0 = std::floor((u0+slack)/period),m1 = std::floor((u1-slack)/period);
    const auto moved = [&](const TopoDS_Face& part,double m) {
        if (m == 0) return part;
        const TopoDS_Face copy = TopoDS::Face(BRepBuilderAPI_Copy(part,true).Shape());
        reseat(copy,surface,-m*period);
        return copy;
    };
    if (m0 == m1) return {moved(face,m0)};
    if (m1 != m0+1) throw std::runtime_error("a ring's piece spans more than a period");
    // the half-plane of the seam line crossed, bounded by the axis
    const double U = m1*period;
    const gp_Pnt on = surface->Value(U,(v0+v1)/2);
    const gp_Lin line(axis);
    const gp_Pnt foot = ElCLib::Value(ElCLib::Parameter(line,on),line);
    const gp_Vec out = gp_Vec(foot,on).Normalized()*reach,along = gp_Vec(axis.Direction())*reach;
    const gp_Pnt o = axis.Location();
    BRepBuilderAPI_MakePolygon polygon(o.Translated(-along),o.Translated(along),o.Translated(along+out),o.Translated(out-along),true);
    BRepBuilderAPI_MakeFace half(polygon.Wire(),true);
    BRepAlgoAPI_Splitter split;
    TopTools_ListOfShape objects,tools;
    objects.Append(face); tools.Append(half.Face());
    split.SetArguments(objects); split.SetTools(tools);
    split.Build();
    check_algorithm(split,"a ring's piece at its seam");
    std::vector<TopoDS_Face> parts;
    for (TopExp_Explorer it(split.Shape(),TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face part = TopoDS::Face(it.Current());
        double a0,a1,b0,b1;
        BRepTools::UVBounds(part,a0,a1,b0,b1);
        const TopoDS_Face own = TopoDS::Face(BRepBuilderAPI_Copy(part,true).Shape());
        reseat(own,surface,0);
        parts.push_back(moved(own,(a0+a1)/2 > U ? m1 : m0));
    }
    if (parts.size() != 2) throw std::runtime_error("a ring's piece split at its seam into "+std::to_string(parts.size())+" faces");
    return parts;
}

// One solid and its copies turned by `angles` about the line (`origin`, `axis`), united: the
// sectors of an indexed body, each meeting the next on a face both carry (docs/native-speed-plan.md).
// The faces on the two sides (`sides`: every sample of the face within `fuzzy` of one of theirs) are
// left out, and the rest of every copy sewn to `fuzzy` (mm) into one closed shell: no face is
// intersected. The pieces of a ring are first moved into one period (`into_one_period`).
int solvent_cad_pattern(Cad* cad,int solid,const double* origin,const double* axis,const double* angles,int count,
    const int* sides,double fuzzy) noexcept {
    return guarded(cad,[&] {
        if (!angles || count < 1 || count > 4096) throw std::runtime_error("a pattern needs 1..4096 turns");
        const bool debug = std::getenv("SOLVENT_SECTOR_DEBUG") != nullptr;
        const auto& source = cad->at(solid);
        const gp_Ax1 line(gp_Pnt(origin[0],origin[1],origin[2]),gp_Dir(axis[0],axis[1],axis[2]));
        double clock = processor_seconds();
        const auto lap = [&](const char* step) {
            if (debug) fprintf(stderr,"sector: pattern: %s %.2f s\n",step,processor_seconds()-clock);
            clock = processor_seconds();
        };
        // which of the sector's faces lie on a side, and on which
        std::vector<Handle(Geom_Surface)> side_surfaces;
        for (int i=0;i<2;++i) for (TopExp_Explorer it(cad->at(sides[i]),TopAbs_FACE); it.More(); it.Next())
            side_surfaces.push_back(BRep_Tool::Surface(TopoDS::Face(it.Current())));
        const auto side_of = [&](const TopoDS_Face& face) {
            double u0,u1,v0,v1;
            BRepTools::UVBounds(face,u0,u1,v0,v1);
            const Handle(Geom_Surface) own = BRep_Tool::Surface(face);
            for (size_t i=0;i<side_surfaces.size();++i) {
                bool all = true;
                for (const auto& [fu,fv]: {std::pair{0.5,0.5},{0.2,0.3},{0.8,0.7},{0.3,0.8}}) {
                    GeomAPI_ProjectPointOnSurf foot(own->Value(u0+(u1-u0)*fu,v0+(v1-v0)*fv),side_surfaces[i]);
                    if (!foot.NbPoints() || foot.LowerDistance() > fuzzy) { all = false; break; }
                }
                if (all) return static_cast<int>(i);
            }
            return -1;
        };
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(source,TopAbs_FACE,faces);
        std::vector<int> side(faces.Extent());
        for (int i=0;i<faces.Extent();++i) side[i] = side_of(TopoDS::Face(faces(i+1)));
        if (std::count(side.begin(),side.end(),0) != 1 || std::count(side.begin(),side.end(),1) != 1)
            throw std::runtime_error("the sector does not carry one face on each side");
        // a ring's piece: a face of revolution meeting both sides
        TopTools_IndexedDataMapOfShapeListOfShape by_edge;
        TopExp::MapShapesAndAncestors(source,TopAbs_EDGE,TopAbs_FACE,by_edge);
        std::vector<char> ring(faces.Extent());
        for (int i=0;i<faces.Extent();++i) {
            const TopoDS_Face face = TopoDS::Face(faces(i+1));
            if (side[i] >= 0 || !about(face,line)) continue;
            bool meets[2] = {false,false};
            for (TopExp_Explorer e(face,TopAbs_EDGE); e.More(); e.Next())
                for (const auto& other: by_edge.FindFromKey(e.Current())) {
                    const int j = faces.FindIndex(other)-1;
                    if (j >= 0 && side[j] >= 0) meets[side[j]] = true;
                }
            ring[i] = meets[0] && meets[1];
        }
        Bnd_Box box;
        BRepBndLib::Add(source,box,false);
        double x0,y0,z0,x1,y1,z1;
        box.Get(x0,y0,z0,x1,y1,z1);
        const double reach = 4.*(std::hypot(std::hypot(x1-x0,y1-y0),z1-z0)+line.Location().Distance(gp_Pnt((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)));
        BRepBuilderAPI_Sewing sewing(fuzzy);
        for (int k=0;k<=count;++k) {
            const TopoDS_Shape copy = k == 0 ? source : turned_copy(source,line,angles[k-1]);
            TopTools_IndexedMapOfShape own;
            TopExp::MapShapes(copy,TopAbs_FACE,own);
            if (own.Extent() != faces.Extent()) throw std::runtime_error("a turned copy has other faces than its source");
            for (int i=0;i<own.Extent();++i) {
                if (side[i] >= 0) continue;
                const TopoDS_Face face = TopoDS::Face(own(i+1));
                if (ring[i]) for (const auto& part: into_one_period(face,line,reach)) sewing.Add(part);
                else sewing.Add(face);
            }
        }
        lap("turned and gathered the copies' faces");
        sewing.Perform();
        TopoDS_Shape sewn = sewing.SewedShape();
        if (sewing.NbFreeEdges() > 0 || sewing.NbMultipleEdges() > 0)
            throw std::runtime_error("the sewn sectors leave "+std::to_string(sewing.NbFreeEdges())+" free and "
                +std::to_string(sewing.NbMultipleEdges())+" multiple edges");
        TopoDS_Shell shell;
        int shells = 0;
        for (TopExp_Explorer it(sewn,TopAbs_SHELL); it.More(); it.Next()) { ++shells; shell = TopoDS::Shell(it.Current()); }
        if (shells != 1) throw std::runtime_error("the sewn sectors make "+std::to_string(shells)+" shells, not one");
        BRepBuilderAPI_MakeSolid solid_of(shell);
        if (!solid_of.IsDone()) throw std::runtime_error("the sewn sectors make no solid");
        TopoDS_Solid made = solid_of.Solid();
        if (!BRepLib::OrientClosedSolid(made)) throw std::runtime_error("the sewn sectors are not closed");
        lap("sewn");
        return united(cad,made,"pattern union");
    });
}

// A solid of revolution about the line (`origin`, `axis`) made again by turning its meridian
// section about that line: every face then carries its surface's own frame on the axis, so copies
// turned about it are one parameterization apart by a turn of `u`, which is what lets faces of
// neighbouring copies merge (a sphere built about another line through its centre would not).
// The section is the solid's common with a half-plane bounded by the axis; the caller compares
// the volumes.
int solvent_cad_revolved(Cad* cad,int solid,const double* origin,const double* axis) noexcept {
    return guarded(cad,[&] {
        const auto& shape = cad->at(solid);
        const gp_Pnt o(origin[0],origin[1],origin[2]);
        const gp_Dir a(axis[0],axis[1],axis[2]);
        const gp_Dir x = std::abs(a.X()) < 0.6 ? gp_Dir(gp_Vec(a).Crossed(gp_Vec(1,0,0))) : gp_Dir(gp_Vec(a).Crossed(gp_Vec(0,1,0)));
        Bnd_Box box;
        BRepBndLib::Add(shape,box,false);
        double x0,y0,z0,x1,y1,z1;
        box.Get(x0,y0,z0,x1,y1,z1);
        const double reach = 2.*(std::hypot(std::hypot(x1-x0,y1-y0),z1-z0)+o.Distance(gp_Pnt((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)));
        const gp_Vec along = gp_Vec(a)*reach, out = gp_Vec(x)*reach;
        BRepBuilderAPI_MakePolygon polygon(o.Translated(-along),o.Translated(along),o.Translated(along+out),o.Translated(out-along),true);
        BRepBuilderAPI_MakeFace half(polygon.Wire(),true);
        if (!half.IsDone()) throw std::runtime_error("the half-plane of a meridian section failed");
        BRepAlgoAPI_Common common(shape,half.Face());
        common.SetFuzzyValue(1e-7);
        common.Build();
        check_algorithm(common,"meridian section");
        TopoDS_Shape result;
        int faces = 0;
        for (TopExp_Explorer it(common.Shape(),TopAbs_FACE); it.More(); it.Next()) {
            ++faces;
            BRepPrimAPI_MakeRevol turned(it.Current(),gp_Ax1(o,a),2*M_PI,true);
            if (!turned.IsDone()) throw std::runtime_error("turning a meridian section failed");
            if (result.IsNull()) result = turned.Shape();
            else {
                BRepAlgoAPI_Fuse fuse(result,turned.Shape());
                check_algorithm(fuse,"turned sections' union");
                result = fuse.Shape();
            }
        }
        if (!faces) throw std::runtime_error("the solid has no meridian section");
        validate(result);
        return cad->put(result);
    });
}

// Classify many points against one solid: 0 outside, 1 inside, 2 within
// tolerance of the boundary. One classifier serves the whole batch, since its
// construction dominates a single query on spline-bounded solids.
int solvent_cad_solid_contains(Cad* cad,int id,const double* points,int count,double tolerance,
    int* output) noexcept {
    return guarded(cad,[&] {
        if (!points || !output || count < 1 || !std::isfinite(tolerance) || tolerance <= 0)
            throw std::runtime_error("solid classification needs points, an output buffer and positive tolerance");
        BRepClass3d_SolidClassifier classify(cad->at(id));
        for (int i=0;i<count;++i) {
            const double* p = points+3*i;
            classify.Perform(gp_Pnt(p[0],p[1],p[2]),tolerance);
            switch (classify.State()) {
            case TopAbs_OUT: output[i] = 0; break;
            case TopAbs_IN: output[i] = 1; break;
            case TopAbs_ON: output[i] = 2; break;
            default: throw std::runtime_error("solid classification is unresolved");
            }
        }
        return 0;
    });
}

// Largest vertex, edge and face tolerance the kernel carries on the shape: the
// geometric slack its topology claims, which any accuracy statement must include.
int solvent_cad_tolerance(Cad* cad,int id,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("tolerance needs an output buffer");
        const auto& shape = cad->at(id);
        output[0] = BRep_Tool::MaxTolerance(shape,TopAbs_VERTEX);
        output[1] = BRep_Tool::MaxTolerance(shape,TopAbs_EDGE);
        output[2] = BRep_Tool::MaxTolerance(shape,TopAbs_FACE);
        return 0;
    });
}

// Volume of the intersection of two shapes, without retaining it.
int solvent_cad_common_volume(Cad* cad,int a,int b,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("common volume needs an output buffer");
        BRepAlgoAPI_Common common(cad->at(a),cad->at(b));
        if (!common.IsDone()) throw std::runtime_error("Boolean intersection failed");
        output[0] = volume(common.Shape());
        return 0;
    });
}

// Rigidly place any shape (a candidate sheet included) by a 3x4 row-major matrix.
// Unlike solid placement this validates nothing beyond the transform itself.
int solvent_cad_place(Cad* cad,int source,const double* matrix) noexcept {
    return guarded(cad,[&] {
        if (!matrix) throw std::runtime_error("placement needs a matrix");
        gp_Trsf pose;
        pose.SetValues(matrix[0],matrix[1],matrix[2],matrix[3],
            matrix[4],matrix[5],matrix[6],matrix[7],matrix[8],matrix[9],matrix[10],matrix[11]);
        BRepBuilderAPI_Transform moved(cad->at(source),pose,true);
        if (!moved.IsDone()) throw std::runtime_error("placement failed");
        return cad->put(moved.Shape());
    });
}

int solvent_cad_volume(Cad* cad,int id,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("volume needs an output buffer");
        output[0] = cad->volume_of(id);
        return 0;
    });
}
}
