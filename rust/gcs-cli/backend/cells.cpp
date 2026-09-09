// Cell decomposition of a stock solid by candidate boundary sheets.
// The kernel arranges; material selection is the caller's, from the source field.
#include "occt.hpp"
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <sstream>

static double volume(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape,props);
    return props.Mass();
}

static void check_algorithm(BRepAlgoAPI_BuilderAlgo& algorithm,const char* what) {
    if (!algorithm.IsDone() || algorithm.HasErrors()) {
        std::ostringstream message;
        message << what << " failed: ";
        algorithm.DumpErrors(message); algorithm.DumpWarnings(message);
        throw std::runtime_error(message.str());
    }
}

extern "C" {
// Partition one solid by faces or solids. Every cell is retained as a separate
// solid; nothing here decides which cells are material. Sheets that end inside
// the stock leave cells unseparated, which the caller's classification must
// detect as a cell whose samples disagree.
int solvent_cad_split_solid(Cad* cad,int solid,const int* tools,int count) noexcept {
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
        split.SetNonDestructive(true); split.SetRunParallel(false);
        split.Build();
        check_algorithm(split,"solid split");
        const auto result = split.Shape();
        int cells = 0;
        for (TopExp_Explorer it(result,TopAbs_SOLID); it.More(); it.Next()) ++cells;
        if (!cells || !BRepCheck_Analyzer(result).IsValid())
            throw std::runtime_error("solid split produced no valid cells");
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
        for (int i=1;i<=count;++i) output[i-1] = cad->put(solids(i));
        return count;
    });
}

// A point strictly inside the solid, with the classifier tolerance at which it
// still reads inside (a lower bound on its distance from the boundary), and the
// volume. The search is a bounded deterministic grid; a thin cell may return a
// small margin, and the caller decides whether that suffices for its field query.
int solvent_cad_solid_sample(Cad* cad,int id,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("solid sample needs an output buffer");
        const auto& shape = cad->at(id);
        if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID)
            throw std::runtime_error("solid sample needs one solid");
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape,props);
        const double v = props.Mass();
        if (!std::isfinite(v) || v <= 0) throw std::runtime_error("cell has no positive volume");
        Bnd_Box box;
        BRepBndLib::Add(shape,box,false);
        double x0,y0,z0,x1,y1,z1;
        box.Get(x0,y0,z0,x1,y1,z1);
        const double diagonal = std::hypot(std::hypot(x1-x0,y1-y0),z1-z0);
        BRepClass3d_SolidClassifier classify(shape);
        const auto inside = [&](const gp_Pnt& p,double margin) {
            classify.Perform(p,margin);
            return classify.State() == TopAbs_IN;
        };
        const int n = 9;
        for (double fraction: {0.1,0.05,0.02,0.01,0.005,0.002,0.001,0.0002}) {
            const double margin = fraction*diagonal;
            const gp_Pnt centroid = props.CentreOfMass();
            if (inside(centroid,margin)) {
                output[0] = centroid.X(); output[1] = centroid.Y(); output[2] = centroid.Z();
                output[3] = margin; output[4] = v;
                return 0;
            }
            for (int i=0;i<n;++i) for (int j=0;j<n;++j) for (int k=0;k<n;++k) {
                const gp_Pnt p(x0+(x1-x0)*(i+0.5)/n,y0+(y1-y0)*(j+0.5)/n,z0+(z1-z0)*(k+0.5)/n);
                if (inside(p,margin)) {
                    output[0] = p.X(); output[1] = p.Y(); output[2] = p.Z();
                    output[3] = margin; output[4] = v;
                    return 0;
                }
            }
        }
        throw std::runtime_error("no interior sample point found in cell");
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
        ShapeUpgrade_UnifySameDomain unify(result,true,true,false);
        unify.Build();
        result = unify.Shape();
        int solids = 0;
        for (TopExp_Explorer it(result,TopAbs_SOLID); it.More(); it.Next()) { ++solids; result = it.Current(); }
        if (solids != 1) throw std::runtime_error("cell union is not one connected solid");
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

int solvent_cad_volume(Cad* cad,int id,double* output) noexcept {
    return guarded(cad,[&] {
        if (!output) throw std::runtime_error("volume needs an output buffer");
        output[0] = volume(cad->at(id));
        return 0;
    });
}
}
