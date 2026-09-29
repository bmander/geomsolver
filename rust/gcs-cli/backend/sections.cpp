// Planar sections of a native solid, as the profile a swept boundary is built over.
#include "occt.hpp"
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <algorithm>
#include <cmath>
#include <vector>

// The last section a thread counted: its arguments and its rows, handed to the call that retrieves it.
static thread_local std::vector<double> section_key;
static thread_local std::vector<int> section_rows;

extern "C" {
// Section a solid by the plane through `origin` containing `axis`, keeping only
// the edges on the half-plane whose in-plane direction is `side` (perpendicular
// to the axis). Each row is [edge handle, face index]: the face the edge lies on
// as its 1-based position in the solid's face enumeration (the order
// solvent_cad_faces uses), which is stable across sections. Chaining the edges
// into loops is the caller's. Count first with rows=null/capacity=0: the section is cut then, and
// the call that retrieves it with the same arguments is handed what that one cut.
int solvent_cad_section(Cad* cad,int solid,const double* origin,const double* axis,const double* side,
    int* rows,int capacity) noexcept {
    return guarded(cad,[&] {
        if (!origin || !axis || !side) throw std::runtime_error("section needs an origin, axis and side");
        const std::vector<double> key{double(solid),origin[0],origin[1],origin[2],axis[0],axis[1],axis[2],side[0],side[1],side[2]};
        if (rows && key == section_key) {
            const int count = static_cast<int>(section_rows.size()/2);
            if (capacity < count) throw std::runtime_error("section buffer is too small");
            std::copy(section_rows.begin(),section_rows.end(),rows);
            section_key.clear();
            return count;
        }
        cad->validated(solid);
        const auto& shape = cad->at(solid);
        const gp_Pnt o(origin[0],origin[1],origin[2]);
        const gp_Dir a(axis[0],axis[1],axis[2]);
        gp_Vec s(side[0],side[1],side[2]);
        s -= gp_Vec(a)*s.Dot(gp_Vec(a));
        if (s.Magnitude() <= 1e-12) throw std::runtime_error("section side must not lie along the axis");
        const gp_Dir side_dir(s);
        // The plane normal is perpendicular to both the axis and the side direction.
        const gp_Pln plane(gp_Ax3(o,gp_Dir(gp_Vec(a).Crossed(gp_Vec(side_dir))),side_dir));
        BRepAlgoAPI_Section section(shape,plane,false);
        section.ComputePCurveOn1(false);
        section.Approximation(false);
        section.SetRunParallel(false);
        // A cutter is sectioned at many stations side by side: the section leaves it as it is.
        section.SetNonDestructive(true);
        section.Build();
        if (!section.IsDone() || section.HasErrors()) throw std::runtime_error("plane section failed");
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape,TopAbs_FACE,faces);
        std::vector<std::pair<TopoDS_Edge,int>> kept;
        for (TopExp_Explorer it(section.Shape(),TopAbs_EDGE); it.More(); it.Next()) {
            const auto whole = TopoDS::Edge(it.Current());
            double t0,t1;
            const auto curve = BRep_Tool::Curve(whole,t0,t1);
            if (curve.IsNull()) throw std::runtime_error("section edge has no curve");
            // An edge of the full plane's section may cross the axis (a face about another
            // axis, or a revolution's own meridian carried over it): split it where it does,
            // keeping the pieces on the side's half-plane.
            const auto offset = [&](double t) { return gp_Vec(o,curve->Value(t)).Dot(gp_Vec(side_dir)); };
            std::vector<double> cuts{t0};
            const int samples = 64;
            for (int k=0;k<samples;++k) {
                double a = t0+(t1-t0)*k/samples, b = t0+(t1-t0)*(k+1)/samples;
                double fa = offset(a), fb = offset(b);
                if ((fa < 0) == (fb < 0)) continue;
                for (int n=0;n<60;++n) {
                    const double m = 0.5*(a+b), fm = offset(m);
                    if ((fm < 0) == (fa < 0)) { a = m; fa = fm; } else { b = m; }
                }
                cuts.push_back(0.5*(a+b));
            }
            cuts.push_back(t1);
            for (size_t c=0;c+1<cuts.size();++c) {
            const double a = cuts[c], b = cuts[c+1];
            if (b-a <= 1e-12*(1.+std::abs(t1-t0))) continue;
            const gp_Pnt middle = curve->Value(0.5*(a+b));
            if (gp_Vec(o,middle).Dot(gp_Vec(side_dir)) <= 0) continue;
            const auto edge = cuts.size() == 2 ? whole : BRepBuilderAPI_MakeEdge(curve,a,b).Edge();
            TopoDS_Shape face;
            int index = 0;
            if (section.HasAncestorFaceOn1(whole,face)) index = faces.FindIndex(face);
            if (!index) {
                // A section edge lying on an existing edge (a seam in the plane)
                // has no recorded ancestor; take the nearest face to its middle.
                const auto vertex = BRepBuilderAPI_MakeVertex(middle).Vertex();
                double best = 1e-6;
                for (int f=1;f<=faces.Extent();++f) {
                    BRepExtrema_DistShapeShape distance(vertex,faces(f));
                    if (distance.IsDone() && distance.NbSolution() > 0 && distance.Value() < best) {
                        best = distance.Value(); index = f;
                    }
                }
                if (!index) throw std::runtime_error("section edge has no source face");
            }
            kept.emplace_back(edge,index);
            }
        }
        const int count = static_cast<int>(kept.size());
        std::vector<int> made(2*kept.size());
        for (int i=0;i<count;++i) { made[2*i] = cad->put(kept[i].first); made[2*i+1] = kept[i].second; }
        if (!rows && capacity == 0) { section_key = key; section_rows = made; return count; }
        if (!rows || capacity < count) throw std::runtime_error("section buffer is too small");
        std::copy(made.begin(),made.end(),rows);
        return count;
    });
}
}
