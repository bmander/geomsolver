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
#include <chrono>
#include <ctime>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
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
#include <array>
#include <cstring>
#include <tuple>
#include <BRepMesh_IncrementalMesh.hxx>
#include <OSD_Parallel.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <limits>
#include <cmath>
#include <vector>
#include <sstream>
#include <functional>
#include <future>
#include <optional>


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
// one closed solid with its volume (`known`, where the caller has measured it another way). Unification is a simplification, not a requirement: keep the
// union when merging faces on one support produces an invalid shape or has to widen a tolerance
// to do so (it once left a 40 mm vertex tolerance). Unify a copy: the algorithm updates
// tolerances on vertices it shares with its input, which would silently widen the union kept.
// `known` is asked only once the union is checked, so a caller may work it out meanwhile.
static int united(Cad* cad,TopoDS_Shape result,const char* what,const std::function<double()>& known) {
    const bool debug = std::getenv("SOLVENT_SECTOR_DEBUG") != nullptr;
    auto clock = std::chrono::steady_clock::now();
    const auto lap = [&](const char* step) {
        const auto now = std::chrono::steady_clock::now();
        if (debug) fprintf(stderr,"sector: %s: %s %.2f s\n",what,step,std::chrono::duration<double>(now-clock).count());
        clock = now;
    };
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
            unified.IsNull() ? -1. : BRep_Tool::MaxTolerance(unified,TopAbs_VERTEX),unified.IsNull() ? -1 : int(valid(unified)));
    }
    // The union checked once: the unified shape where it is kept, the union itself where not.
    const auto checked = [](const TopoDS_Shape& shape) {
        TopoDS_Shape one;
        int solids = 0;
        for (TopExp_Explorer it(shape,TopAbs_SOLID); it.More(); it.Next()) { ++solids; one = it.Current(); }
        return solids == 1 ? valid_solid(one) : valid(shape);
    };
    const bool kept = !unified.IsNull() && BRep_Tool::MaxTolerance(unified,TopAbs_VERTEX) <= before*1.001+1e-9 && checked(unified);
    if (kept) result = unified;
    else if (!checked(result)) throw std::runtime_error(std::string(what)+" is invalid before unification");
    lap("checked");
    int solids = 0;
    for (TopExp_Explorer it(result,TopAbs_SOLID); it.More(); it.Next()) { ++solids; result = it.Current(); }
    if (solids != 1) throw std::runtime_error(std::string(what)+" is not one connected solid");
    const double v = validate(result,nullptr,true,known());
    lap("validated and measured");
    return cad->put(result,true,v);
}
static int united(Cad* cad,TopoDS_Shape result,const char* what,double known = std::numeric_limits<double>::quiet_NaN()) {
    return united(cad,result,what,std::function<double()>([known] { return known; }));
}

// A partition's cells checked and measured, the work shared out: every face the cells have checked
// once, on every core, with its own analyzer (`valid_solid`'s face check), and each cell's shell
// closed and oriented; every face's flux about one point integrated once, on every core, and each
// cell's volume the sum of its faces' fluxes signed by how the cell holds them — a face two cells
// share bounds the one forward and the other reversed. The same checks and the same integrals,
// each done once where `validate` did them once a cell. Throws as `validate` does; records each
// cell's volume. `SOLVENT_FULL_CHECK` checks and measures the cells as `validate` does.
static void validate_cells(TopoDS_Shape& partition,TopTools_DataMapOfShapeReal& record) {
    if (std::getenv("SOLVENT_FULL_CHECK")) { validate(partition,&record); return; }
    std::vector<TopoDS_Solid> cells;
    for (TopExp_Explorer it(partition,TopAbs_SOLID); it.More(); it.Next()) cells.push_back(TopoDS::Solid(it.Current()));
    if (cells.empty()) throw std::runtime_error("operation produced no solid");
    for (const auto& cell: cells) {
        int shells = 0;
        for (TopExp_Explorer it(cell,TopAbs_SHELL); it.More(); it.Next()) ++shells;
        if (shells != 1) { validate(partition,&record); return; }
    }
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(partition,TopAbs_FACE,faces);
    std::vector<char> ok(static_cast<size_t>(faces.Extent()),0);
    OSD_Parallel::For(0,faces.Extent(),[&](int i) { ok[static_cast<size_t>(i)] = BRepCheck_Analyzer(faces(i+1)).IsValid(); });
    if (std::find(ok.begin(),ok.end(),0) != ok.end()) throw std::runtime_error("native solid is invalid:"+invalidity(partition));
    // each cell closed and consistently oriented, then turned to hold its material (as `validate`)
    for (auto& cell: cells) {
        if (!valid_solid(cell)) throw std::runtime_error("native solid is invalid:"+invalidity(cell));
        if (!BRepLib::OrientClosedSolid(cell)) throw std::runtime_error("solid is open");
    }
    // one point for every flux, as `volume` takes it: the mean of the partition's vertices
    gp_XYZ sum(0,0,0);
    int count = 0;
    for (TopExp_Explorer it(partition,TopAbs_VERTEX); it.More(); it.Next(),++count) sum += BRep_Tool::Pnt(TopoDS::Vertex(it.Current())).XYZ();
    if (count > 0) sum /= count;
    std::vector<TopoDS_Face> forward;
    for (int i=1;i<=faces.Extent();++i) forward.push_back(TopoDS::Face(faces(i).Oriented(TopAbs_FORWARD)));
    std::vector<double> mass(forward.size(),0.);
    OSD_Parallel::For(0,static_cast<int>(forward.size()),[&](int i) { mass[static_cast<size_t>(i)] = flux({forward[static_cast<size_t>(i)]},gp_Pnt(sum)); });
    for (const auto& cell: cells) {
        double v = 0;
        for (TopExp_Explorer it(cell,TopAbs_FACE); it.More(); it.Next()) {
            const TopAbs_Orientation o = it.Current().Orientation();
            if (o != TopAbs_FORWARD && o != TopAbs_REVERSED) continue;
            const double m = mass[static_cast<size_t>(faces.FindIndex(it.Current())-1)];
            v += o == TopAbs_FORWARD ? m : -m;
        }
        if (!std::isfinite(v) || v <= 0) throw std::runtime_error("solid has no positive volume");
        record.Bind(cell,v);
    }
    // the cells, oriented, are the partition's
    BRep_Builder builder;
    TopoDS_Compound whole;
    builder.MakeCompound(whole);
    for (const auto& cell: cells) builder.Add(whole,cell);
    partition = whole;
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
        if (stock.IsNull() || !valid(stock))
            throw std::runtime_error("invalid stock solid");
        TopTools_ListOfShape objects,cutters;
        objects.Append(stock);
        for (int i=0;i<count;++i) {
            const auto& tool = cad->at(tools[i]);
            if (tool.IsNull() || !valid(tool))
                throw std::runtime_error("invalid split tool");
            cutters.Append(tool);
        }
        BRepAlgoAPI_Splitter split;
        split.SetArguments(objects); split.SetTools(cutters);
        // Oriented boxes cull face pairs the axis-aligned ones cannot (the tilted sheets):
        // a quarter of the split, and the same cells.
        // Its intersections on every core (`SOLVENT_PARALLEL_SPLIT=off`: one).
        const char* serial = std::getenv("SOLVENT_PARALLEL_SPLIT");
        const bool parallel = !serial || std::string(serial) != "off";
        split.SetNonDestructive(true); split.SetRunParallel(parallel); split.SetUseOBB(Standard_True);
        if (fuzzy > 0) split.SetFuzzyValue(fuzzy);
        // The kernel polls the break only between its phases, so a stop can come well after the
        // budget; the message says both. Processor time is every thread's, so a split run on every
        // core is given each core's budget.
        const double budget = (15.+5.*count)*(parallel ? std::max(1u,std::thread::hardware_concurrency()) : 1u);
        const double started = processor_seconds();
        const auto clock = std::chrono::steady_clock::now();
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
        const auto built = std::chrono::steady_clock::now();
        // Every cell's volume is measured here; the cells, listed next, keep it.
        TopTools_DataMapOfShapeReal measured;
        validate_cells(checked,measured);
        if (std::getenv("SOLVENT_SECTOR_DEBUG")) fprintf(stderr,"split: %d cells, built %.2f s, checked and measured %.2f s\n",cells,
            std::chrono::duration<double>(built-clock).count(),std::chrono::duration<double>(std::chrono::steady_clock::now()-built).count());
        cad->record(measured);
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
            double volume = std::numeric_limits<double>::quiet_NaN();
            cad->recorded(solids(i),volume);
            output[i-1] = cad->put(solids(i),false,volume);
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
static int cell_samples(const TopoDS_Shape& shape,double* output,int capacity,int measure) {
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
}
// `cell_samples` of several cells, each on its own core: `output` holds `capacity` rows of four a
// cell, `written` how many of them each cell filled.
int solvent_cad_solids_samples(Cad* cad,const int* ids,int count,double* output,int capacity,int measure,int* written) noexcept {
    return guarded(cad,[&] {
        if (!ids || !output || !written || count < 0 || capacity < 1 || measure < 1)
            throw std::runtime_error("solid samples need buffers and positive counts");
        std::vector<TopoDS_Shape> cells;
        for (int i=0;i<count;++i) cells.push_back(cad->at(ids[i]));
        std::vector<std::string> failed(static_cast<size_t>(count));
        OSD_Parallel::For(0,count,[&](int i) {
            try { written[i] = cell_samples(cells[static_cast<size_t>(i)],output+4*capacity*i,capacity,measure); }
            catch (const Standard_Failure& e) { failed[static_cast<size_t>(i)] = e.GetMessageString(); }
            catch (const std::exception& e) { failed[static_cast<size_t>(i)] = e.what(); }
            catch (...) { failed[static_cast<size_t>(i)] = "unknown native CAD exception"; }
        });
        for (const auto& message: failed) if (!message.empty()) throw std::runtime_error(message);
        return count;
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
        // one cell is its own union, and its volume, where it was measured with the partition, is known
        double known = std::numeric_limits<double>::quiet_NaN();
        if (count == 1) { result = cad->at(ids[0]); known = cad->known_volume(ids[0]); }
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
        return united(cad,result,"cell union",known);
    });
}

// A face of revolution about `axis`: an elementary surface (not a plane) whose own axis is that
// line, either way along it. Only these are carried onto one surface and one period.
// The surface a face lies on, unwrapped from the trimming a split puts about a face crossing its seam.
static Handle(Geom_Surface) basis(const TopoDS_Face& face,TopLoc_Location& there) {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face,there);
    while (auto trimmed = Handle(Geom_RectangularTrimmedSurface)::DownCast(surface)) surface = trimmed->BasisSurface();
    return surface;
}
static Handle(Geom_Surface) basis(const TopoDS_Face& face) { TopLoc_Location there; return basis(face,there); }

static bool about(const TopoDS_Face& face,const gp_Ax1& axis) {
    TopLoc_Location there;
    const auto elementary = Handle(Geom_ElementarySurface)::DownCast(basis(face,there));
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

// A face on a B-spline surface put on the surface cut down to the face's parameter box (and a
// thousandth of the surface's range about it): `Geom_BSplineSurface::Segment` keeps the parameters,
// so the face's pcurves stand as they are and the face is the same face, but a file writing it
// carries the piece of the sheet the face uses, not the whole sheet the construction fitted.
static void restrict_to_face(const TopoDS_Face& face) {
    TopLoc_Location there;
    const auto spline = Handle(Geom_BSplineSurface)::DownCast(basis(face,there));
    if (spline.IsNull() || !there.IsIdentity() || spline->IsUPeriodic() || spline->IsVPeriodic()) return;
    double u0,u1,v0,v1,U0,U1,V0,V1;
    BRepTools::UVBounds(face,u0,u1,v0,v1);
    spline->Bounds(U0,U1,V0,V1);
    const double du = (U1-U0)*1e-3,dv = (V1-V0)*1e-3;
    const double a = std::max(U0,u0-du),b = std::min(U1,u1+du),c = std::max(V0,v0-dv),d = std::min(V1,v1+dv);
    if (!(a < b && c < d) || (a <= U0 && b >= U1 && c <= V0 && d >= V1)) return;
    const auto cut = Handle(Geom_BSplineSurface)::DownCast(spline->Copy());
    cut->Segment(a,b,c,d);
    reseat(face,cut,0.);
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
        const Handle(Geom_Surface) surface = basis(original);
        // which way the parameter runs about the axis: the turn is a shift of u by +angle or -angle
        double u0,u1,v0,v1;
        BRepTools::UVBounds(original,u0,u1,v0,v1);
        const gp_Pnt p = surface->Value(u0,v0).Transformed(turn);
        double shift = std::nan("");
        for (const double sign: {1.,-1.})
            if (surface->Value(u0+sign*angle,v0).Distance(p) <= 1e-9*(1.+p.Distance(gp_Pnt(0,0,0)))) { shift = sign*angle; break; }
        if (std::isnan(shift)) throw std::runtime_error("a turned face's parameters are not its surface's turned");
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
    const Handle(Geom_Surface) surface = basis(face);
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
    // The kernel finds no crossing on a face whose parameters run past the period's end, so the
    // face is split on its surface turned half a turn (whose u is u - U + π there), and put back.
    const Handle(Geom_Surface) turned = Handle(Geom_Surface)::DownCast(surface->Rotated(axis,M_PI));
    const TopoDS_Face across = TopoDS::Face(BRepBuilderAPI_Copy(face,true).Shape());
    reseat(across,turned,M_PI-U);
    BRepAlgoAPI_Splitter split;
    TopTools_ListOfShape objects,tools;
    objects.Append(across); tools.Append(half.Face());
    split.SetArguments(objects); split.SetTools(tools);
    split.Build();
    check_algorithm(split,"a ring's piece at its seam");
    std::vector<TopoDS_Face> parts;
    for (TopExp_Explorer it(split.Shape(),TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face part = TopoDS::Face(it.Current());
        double a0,a1,b0,b1;
        BRepTools::UVBounds(part,a0,a1,b0,b1);
        const TopoDS_Face own = TopoDS::Face(BRepBuilderAPI_Copy(part,true).Shape());
        reseat(own,surface,U-M_PI);
        parts.push_back(moved(own,(a0+a1)/2 > M_PI ? m1 : m0));
    }
    if (parts.size() != 2) {
        std::ostringstream message;
        message << "a ring's piece (" << surface->DynamicType()->Name() << ", u " << u0 << " to " << u1 << ", v " << v0 << " to " << v1
            << ") split at its seam (u " << U << ") into " << parts.size() << " faces";
        throw std::runtime_error(message.str());
    }
    return parts;
}

// Which of a sector's faces (`faces`) lie on one of its two sides (`sides`, two shapes), and on
// which: 0 or 1 where every sample of the face is within `fuzzy` of that side's surface, -1 else.
static std::vector<int> sector_sides(Cad* cad,const TopTools_IndexedMapOfShape& faces,const int* sides,double fuzzy) {
    std::vector<std::pair<int,Handle(Geom_Surface)>> side_surfaces;
    for (int i=0;i<2;++i) for (TopExp_Explorer it(cad->at(sides[i]),TopAbs_FACE); it.More(); it.Next())
        side_surfaces.emplace_back(i,BRep_Tool::Surface(TopoDS::Face(it.Current())));
    const auto side_of = [&](const TopoDS_Face& face) {
        double u0,u1,v0,v1;
        BRepTools::UVBounds(face,u0,u1,v0,v1);
        const Handle(Geom_Surface) own = BRep_Tool::Surface(face);
        for (const auto& [which,surface]: side_surfaces) {
            bool all = true;
            for (const auto& [fu,fv]: {std::pair{0.5,0.5},{0.2,0.3},{0.8,0.7},{0.3,0.8}}) {
                GeomAPI_ProjectPointOnSurf foot(own->Value(u0+(u1-u0)*fu,v0+(v1-v0)*fv),surface);
                if (!foot.NbPoints() || foot.LowerDistance() > fuzzy) { all = false; break; }
            }
            if (all) return which;
        }
        return -1;
    };
    std::vector<int> side(faces.Extent());
    for (int i=0;i<faces.Extent();++i) side[i] = side_of(TopoDS::Face(faces(i+1)));
    return side;
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
        auto clock = std::chrono::steady_clock::now();
        const auto lap = [&](const char* step) {
            const auto now = std::chrono::steady_clock::now();
            if (debug) fprintf(stderr,"sector: pattern: %s %.2f s\n",step,std::chrono::duration<double>(now-clock).count());
            clock = now;
        };
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(source,TopAbs_FACE,faces);
        const std::vector<int> side = sector_sides(cad,faces,sides,fuzzy);
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
        // The sector on its surfaces as the copies will be (a face a split left on a trimmed surface
        // too), each face of revolution given the parameters within half a turn of the sector's middle:
        // a face's pieces in neighbouring copies then continue one another's, where a piece a period
        // away would meet its neighbour on pcurves a period apart, and never merge.
        const TopoDS_Shape base = turned_copy(source,line,0.);
        const gp_Pnt middle((x0+x1)/2,(y0+y1)/2,(z0+z1)/2);
        // Every copy's volume is the base's faces' but the sides' flux about a point of the axis,
        // which a turn about the axis keeps: the sides are left out of the union and the rest of
        // every copy is sewn as it is.
        std::vector<TopoDS_Face> kept;
        {
            TopTools_IndexedMapOfShape own;
            TopExp::MapShapes(base,TopAbs_FACE,own);
            for (int i=0;i<own.Extent();++i) if (side[static_cast<size_t>(i)] < 0) {
                const TopoDS_Face face = TopoDS::Face(own(i+1));
                restrict_to_face(face);
                kept.push_back(face);
            }
            for (TopExp_Explorer it(base,TopAbs_FACE); it.More(); it.Next()) {
                const TopoDS_Face face = TopoDS::Face(it.Current());
                if (!about(face,line)) continue;
                const Handle(Geom_Surface) surface = basis(face);
                GeomAPI_ProjectPointOnSurf foot(middle,surface);
                if (!foot.NbPoints()) throw std::runtime_error("the sector's middle has no foot on a face of revolution");
                double uc,vc,u0,u1,v0,v1;
                foot.LowerDistanceParameters(uc,vc);
                BRepTools::UVBounds(face,u0,u1,v0,v1);
                const double m = std::round(((u0+u1)/2-uc)/(2*M_PI));
                if (m != 0) reseat(face,surface,-m*2*M_PI);
            }
        }
        BRepBuilderAPI_Sewing sewing(fuzzy);
        for (int k=0;k<=count;++k) {
            const TopoDS_Shape copy = k == 0 ? base : turned_copy(base,line,angles[k-1]);
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
        const gp_Lin axis_line(line);
        // A copy's faces' flux about a point of the axis, measured while the union is unified and
        // checked, and waited for as it is stored.
        std::future<double> each = std::async(std::launch::async,[&] {
            return flux(kept,ElCLib::Value(ElCLib::Parameter(axis_line,middle),axis_line));
        });
        std::optional<double> whole;
        const auto whole_of = [&] {
            if (!whole) { whole = (count+1)*each.get(); lap("measured one copy"); }
            return *whole;
        };
        // `SOLVENT_SECTOR_CHECK=full`: the united solid measured whole as well, which may differ by
        // the slack its tolerances leave the boundary (the sewing moves edges within them).
        if (std::getenv("SOLVENT_SECTOR_CHECK") && std::string(std::getenv("SOLVENT_SECTOR_CHECK")) == "full") {
            const double measured = volume(made);
            const double slack = std::max(1e-8*std::abs(measured),BRep_Tool::MaxTolerance(made,TopAbs_VERTEX)*area(made));
            if (debug) fprintf(stderr,"sector: pattern: %.12g mm3 measured whole, %.12g as %d copies of one (slack %.3g)\n",
                measured,whole_of(),count+1,slack);
            if (std::abs(measured-whole_of()) > slack)
                throw std::runtime_error("the sectors united measure "+std::to_string(measured)+" mm3 whole and "+std::to_string(whole_of())
                    +" as copies of one");
        }
        const int id = united(cad,made,"pattern union",std::function<double()>(whole_of));
        cad->set_pattern(id,line,count+1);
        return id;
    });
}

// Mesh a sector's material (the source `solvent_cad_pattern` turns) afresh, its faces on every
// core, at an absolute `deflection` (mm) and an `angular` one (radians), and read the chordal sag
// its faces have, the two sides' (`sides`, found as the pattern finds them) left out, since their
// triangles are no part of the pattern's mesh. `output`: the sag (mm) and where; none, no sag read.
int solvent_cad_sector_mesh(Cad* cad,int piece,const int* sides,double fuzzy,double deflection,double angular,
    double* output) noexcept {
    return guarded(cad,[&] {
        if (!sides) throw std::runtime_error("a sector's mesh needs its sides");
        if (!std::isfinite(deflection) || deflection <= 0 || !std::isfinite(angular) || angular <= 0)
            throw std::runtime_error("meshing needs a positive deflection and angle");
        cad->validated(piece);
        auto& shape = cad->at(piece);
        BRepTools::Clean(shape);
        BRepMesh_IncrementalMesh mesher(shape,deflection,false,angular,true);
        if (!mesher.IsDone()) throw std::runtime_error("native tessellation failed");
        if (!output) return 0;
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape,TopAbs_FACE,faces);
        const std::vector<int> side = sector_sides(cad,faces,sides,fuzzy);
        std::vector<double> worst(faces.Extent(),0.);
        std::vector<gp_Pnt> at(faces.Extent());
        OSD_Parallel::For(0,faces.Extent(),[&](int i) {
            if (side[static_cast<size_t>(i)] < 0) face_sag(TopoDS::Face(faces(i+1)),worst[static_cast<size_t>(i)],at[static_cast<size_t>(i)]);
        });
        output[0] = 0;
        for (int i=0;i<faces.Extent();++i) if (worst[static_cast<size_t>(i)] > output[0]) {
            output[0] = worst[static_cast<size_t>(i)]; output[1] = at[static_cast<size_t>(i)].X();
            output[2] = at[static_cast<size_t>(i)].Y(); output[3] = at[static_cast<size_t>(i)].Z();
        }
        return 0;
    });
}

// The binary STL of a sector meshed by `solvent_cad_sector_mesh` and turned `count` times by
// `pitch` about the line (`origin`, `axis`): every triangle but the sides', copy k's the sector's
// turned by k pitches. A copy meets its neighbour on the neighbour's side, and there the two write
// one set of points: each node where a face meets one side is paired, one to one, with the nearest
// turn of a node where a face meets the other (the sides being a pitch's turn of each other) —
// within `reach` (mm) and a quarter of the least spacing between the seam's points, or the call
// refuses — and a copy writes such a node as its partner turned
// into the neighbouring copy, computed as that copy computes it, so the copies share their seam
// points bit for bit and close into one shell. The mesher discretizes the two sides' edges alike
// but not always at the same places along them (the split cuts each side's edges on its own); the
// partner's turn is on the same two surfaces, so the move is along the seam. `output` gets the
// farthest a node is moved to its partner (mm). Returns the triangles written.
int solvent_cad_sector_stl(Cad* cad,int piece,const int* sides,double fuzzy,const double* origin,const double* axis,
    int count,double pitch,double reach,const char* path,double* output) noexcept {
    return guarded(cad,[&] {
        if (!sides || !origin || !axis || !path || !output || count < 1 || !std::isfinite(pitch) || !(reach > 0))
            throw std::runtime_error("a sector's STL needs its sides, an axis, a count, a pitch, a reach, a path and an output");
        const auto& shape = cad->at(piece);
        const gp_Ax1 line(gp_Pnt(origin[0],origin[1],origin[2]),gp_Dir(axis[0],axis[1],axis[2]));
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape,TopAbs_FACE,faces);
        const std::vector<int> side = sector_sides(cad,faces,sides,fuzzy);
        TopTools_IndexedDataMapOfShapeListOfShape by_edge;
        TopExp::MapShapesAndAncestors(shape,TopAbs_EDGE,TopAbs_FACE,by_edge);
        // Every kept face's nodes (placed), its triangles wound outward, and which side each node is
        // on (-1 for neither).
        struct Mesh { std::vector<gp_Pnt> nodes; std::vector<int> on; std::vector<std::array<int,3>> triangles; };
        std::vector<Mesh> meshes;
        std::vector<gp_Pnt> seam[2];
        for (int i=0;i<faces.Extent();++i) {
            if (side[static_cast<size_t>(i)] >= 0) continue;
            const TopoDS_Face face = TopoDS::Face(faces(i+1));
            TopLoc_Location location;
            const auto triangulation = BRep_Tool::Triangulation(face,location);
            if (triangulation.IsNull() || triangulation->NbTriangles() == 0) throw std::runtime_error("a sector's face is not meshed");
            const gp_Trsf placed = location.Transformation();
            Mesh mesh;
            for (int n=1;n<=triangulation->NbNodes();++n) mesh.nodes.push_back(triangulation->Node(n).Transformed(placed));
            mesh.on.assign(mesh.nodes.size(),-1);
            for (TopExp_Explorer e(face,TopAbs_EDGE); e.More(); e.Next()) {
                int s = -1;
                for (const auto& other: by_edge.FindFromKey(e.Current())) {
                    const int j = faces.FindIndex(other)-1;
                    if (j >= 0 && side[static_cast<size_t>(j)] >= 0) s = side[static_cast<size_t>(j)];
                }
                if (s < 0) continue;
                TopLoc_Location at;
                const auto polygon = BRep_Tool::PolygonOnTriangulation(TopoDS::Edge(e.Current()),triangulation,at);
                if (polygon.IsNull()) throw std::runtime_error("a sector's seam edge has no polygon on its face's mesh");
                for (int k=1;k<=polygon->NbNodes();++k) {
                    const int n = polygon->Node(k)-1;
                    if (mesh.on[static_cast<size_t>(n)] < 0) { mesh.on[static_cast<size_t>(n)] = s; seam[s].push_back(mesh.nodes[static_cast<size_t>(n)]); }
                }
            }
            const bool reversed = face.Orientation() == TopAbs_REVERSED;
            for (int t=1;t<=triangulation->NbTriangles();++t) {
                int a,b,c; triangulation->Triangle(t).Get(a,b,c);
                if (reversed) std::swap(b,c);
                mesh.triangles.push_back({a-1,b-1,c-1});
            }
            meshes.push_back(std::move(mesh));
        }
        // The seam points, each once (a node where faces meet is in each face's mesh).
        const auto distinct = [](std::vector<gp_Pnt>& points) {
            std::sort(points.begin(),points.end(),[](const gp_Pnt& p,const gp_Pnt& q) {
                return std::make_tuple(p.X(),p.Y(),p.Z()) < std::make_tuple(q.X(),q.Y(),q.Z()); });
            points.erase(std::unique(points.begin(),points.end(),[](const gp_Pnt& p,const gp_Pnt& q) {
                return p.X() == q.X() && p.Y() == q.Y() && p.Z() == q.Z(); }),points.end());
        };
        distinct(seam[0]); distinct(seam[1]);
        if (seam[0].size() != seam[1].size() || seam[0].empty())
            throw std::runtime_error("the sector's mesh has "+std::to_string(seam[0].size())+" points on one side and "
                +std::to_string(seam[1].size())+" on the other");
        // Which way a pitch turns the first side onto the second, and each second-side point's partner.
        const auto turn = [&](double angle) { gp_Trsf t; if (angle != 0) t.SetRotation(line,angle); return t; };
        // A partner nearer than a quarter of the nearest two seam points are to each other is no
        // other point's: the pairing is the seam's own order, whatever the mesher did along it.
        double spacing = 1e300;
        for (size_t v=0;v<seam[0].size();++v) for (size_t u=v+1;u<seam[0].size();++u) spacing = std::min(spacing,seam[0][v].Distance(seam[0][u]));
        reach = std::min(reach,spacing/4);
        int sense = 0;
        double moved[2] = {0,0};
        std::vector<size_t> partner(seam[1].size());
        for (const int trial: {1,-1}) {
            const gp_Trsf by = turn(trial*pitch);
            std::vector<gp_Pnt> turned(seam[0].size());
            for (size_t v=0;v<seam[0].size();++v) turned[v] = seam[0][v].Transformed(by);
            std::vector<char> taken(seam[0].size(),0);
            bool all = true;
            double& most = moved[trial > 0 ? 0 : 1];
            for (size_t w=0;w<seam[1].size() && all;++w) {
                double best = 1e300; size_t found = 0;
                for (size_t v=0;v<seam[0].size();++v) {
                    const double d = turned[v].Distance(seam[1][w]);
                    if (d < best) { best = d; found = v; }
                }
                most = std::max(most,best);
                if (best > reach || taken[found]) all = false;
                else { taken[found] = 1; partner[w] = found; }
            }
            if (all) { sense = trial; output[0] = most; break; }
        }
        if (!sense) {
            std::ostringstream message;
            message << "the sector's mesh points on its two sides are not a pitch's turn of each other within " << reach
                << " mm (" << seam[0].size() << " points a side; one " << moved[0] << " mm from its nearest partner turned one way, "
                << moved[1] << " the other)";
            throw std::runtime_error(message.str());
        }
        // A node at a second-side seam point, in whichever face's mesh (a face meeting the side at a
        // corner only has one too): that point's index, or none.
        const auto index_of = [&](const gp_Pnt& p) {
            const auto it = std::lower_bound(seam[1].begin(),seam[1].end(),p,[](const gp_Pnt& a,const gp_Pnt& b) {
                return std::make_tuple(a.X(),a.Y(),a.Z()) < std::make_tuple(b.X(),b.Y(),b.Z()); });
            if (it == seam[1].end() || it->X() != p.X() || it->Y() != p.Y() || it->Z() != p.Z()) return seam[1].size();
            return static_cast<size_t>(it-seam[1].begin());
        };
        std::vector<gp_Trsf> copies(static_cast<size_t>(count));
        for (int k=0;k<count;++k) copies[static_cast<size_t>(k)] = turn(k*pitch);
        size_t triangles = 0;
        for (const auto& mesh: meshes) triangles += mesh.triangles.size();
        triangles *= static_cast<size_t>(count);
        if (triangles > 0xffffffffu) throw std::runtime_error("too many triangles for a binary STL");
        std::vector<char> bytes(84+50*triangles,0);
        const uint32_t n32 = static_cast<uint32_t>(triangles);
        std::memcpy(bytes.data()+80,&n32,4);
        std::vector<std::vector<size_t>> second(meshes.size());
        for (size_t m=0;m<meshes.size();++m) for (const auto& p: meshes[m].nodes) second[m].push_back(index_of(p));
        size_t at = 84;
        for (int k=0;k<count;++k) for (size_t m=0;m<meshes.size();++m) {
            const auto& mesh = meshes[m];
            // each node of this copy, as float32
            std::vector<std::array<float,3>> written(mesh.nodes.size());
            for (size_t n=0;n<mesh.nodes.size();++n) {
                gp_Pnt p = mesh.nodes[n];
                const gp_Trsf* by = &copies[static_cast<size_t>(k)];
                if (const size_t w = second[m][n]; w < seam[1].size()) {
                    p = seam[0][partner[w]];
                    by = &copies[static_cast<size_t>(((k+sense)%count+count)%count)];
                }
                if (by->Form() != gp_Identity) p.Transform(*by);
                written[n] = {static_cast<float>(p.X()),static_cast<float>(p.Y()),static_cast<float>(p.Z())};
            }
            for (const auto& t: mesh.triangles) {
                const auto& a = written[static_cast<size_t>(t[0])];
                const auto& b = written[static_cast<size_t>(t[1])];
                const auto& c = written[static_cast<size_t>(t[2])];
                gp_Vec normal = gp_Vec(b[0]-a[0],b[1]-a[1],b[2]-a[2]).Crossed(gp_Vec(c[0]-a[0],c[1]-a[1],c[2]-a[2]));
                if (normal.Magnitude() > 0) normal.Normalize();
                const float row[12] = {float(normal.X()),float(normal.Y()),float(normal.Z()),a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2]};
                std::memcpy(bytes.data()+at,row,48);
                at += 50;
            }
        }
        std::FILE* file = std::fopen(path,"wb");
        if (!file) throw std::runtime_error(std::string("cannot write ")+path);
        const size_t written = std::fwrite(bytes.data(),1,bytes.size(),file);
        if (std::fclose(file) != 0 || written != bytes.size()) throw std::runtime_error(std::string("STL write failed: ")+path);
        return static_cast<int>(std::min<size_t>(triangles,0x7fffffff));
    });
}

// A solid of revolution about the line (`origin`, `axis`) made again by turning its meridian
// section about that line: every face then carries its surface's own frame on the axis, so copies
// turned about it are one parameterization apart by a turn of `u`, which is what lets faces of
// neighbouring copies merge (a sphere built about another line through its centre would not).
// The section is the solid's common with the half-plane bounded by the axis towards `seam`, which is
// where the faces' parameters start; the caller compares the volumes.
int solvent_cad_revolved(Cad* cad,int solid,const double* origin,const double* axis,const double* seam) noexcept {
    return guarded(cad,[&] {
        const auto& shape = cad->at(solid);
        const gp_Pnt o(origin[0],origin[1],origin[2]);
        const gp_Dir a(axis[0],axis[1],axis[2]);
        const gp_Vec towards(seam[0],seam[1],seam[2]);
        const gp_Dir x(towards-gp_Vec(a)*towards.Dot(gp_Vec(a)));
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
        // Non-destructively: the blank is read by every sweep's clearance, side by side.
        BRepAlgoAPI_Common common;
        TopTools_ListOfShape objects,tools;
        objects.Append(cad->at(a)); tools.Append(cad->at(b));
        common.SetArguments(objects); common.SetTools(tools);
        common.SetNonDestructive(true);
        common.Build();
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
