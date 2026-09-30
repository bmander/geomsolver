// The actual trimmed topology of a constructed solid, including Boolean edges.
#include "occt.hpp"
#include "probe.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <cmath>

extern "C" {
// Count first with rows=null, capacity=0. Then supply capacity rows of four ints:
// edge handle, first face handle, second face handle, flags (1=seam, 2=degenerate).
// Every topological edge is retained, including seams and collapsed pole edges.
int solvent_cad_boundary(Cad* cad,int source,int* rows,int capacity) noexcept {
    SOLVENT_PROBE("solvent_cad_boundary");
    return guarded(cad,[&] {
        auto solid = cad->at(source);
        validate(solid);
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(solid,TopAbs_FACE,faces);
        TopTools_IndexedDataMapOfShapeListOfShape edges;
        TopExp::MapShapesAndUniqueAncestors(solid,TopAbs_EDGE,TopAbs_FACE,edges);
        const int count = edges.Extent();
        if (!rows && capacity == 0) return count;
        if (!rows || capacity < count) throw std::runtime_error("boundary buffer is too small");
        std::vector<int> face_ids;
        for (int i=1;i<=faces.Extent();++i) face_ids.push_back(cad->put(faces(i)));
        for (int i=1;i<=count;++i) {
            const auto edge = TopoDS::Edge(edges.FindKey(i));
            const auto& adjacent = edges.FindFromIndex(i);
            if (adjacent.Extent() < 1 || adjacent.Extent() > 2)
                throw std::runtime_error("boundary edge does not have manifold face incidence");
            const auto a = TopoDS::Face(adjacent.First());
            const auto b = TopoDS::Face(adjacent.Last());
            const bool seam = adjacent.Extent() == 1 && BRep_Tool::IsClosed(edge,a);
            const bool degenerate = BRep_Tool::Degenerated(edge);
            if (adjacent.Extent() == 1 && !seam && !degenerate)
                throw std::runtime_error("boundary contains an open edge");
            rows[4*(i-1)] = cad->put(edge);
            rows[4*(i-1)+1] = face_ids.at(faces.FindIndex(a)-1);
            rows[4*(i-1)+2] = face_ids.at(faces.FindIndex(b)-1);
            rows[4*(i-1)+3] = (seam ? 1 : 0) | (degenerate ? 2 : 0);
        }
        return count;
    });
}

// Sample one boundary row. Output: position, unit curve tangent, two outward
// face normals, two face/curve discrepancies, and signed dihedral (15 doubles).
// Negative dihedral means convex, positive means concave, zero means smooth.
// The curve tangent follows increasing native parameter, not wire orientation.
int solvent_cad_boundary_point(Cad* cad,const int* row,double fraction,double* output) noexcept {
    SOLVENT_PROBE("solvent_cad_boundary_point");
    return guarded(cad,[&] {
        if (!row || !output || !std::isfinite(fraction) || fraction < 0 || fraction > 1)
            throw std::runtime_error("boundary query parameter must lie in [0,1]");
        const auto edge = TopoDS::Edge(cad->at(row[0]).Oriented(TopAbs_FORWARD));
        if (BRep_Tool::Degenerated(edge)) throw std::runtime_error("collapsed boundary edge has no tangent");
        if (!BRep_Tool::SameParameter(edge) || !BRep_Tool::SameRange(edge))
            throw std::runtime_error("boundary edge lacks same-parameter curves");
        BRepAdaptor_Curve curve(edge);
        const double lo = curve.FirstParameter(),hi = curve.LastParameter();
        if (!std::isfinite(lo) || !std::isfinite(hi) || hi <= lo)
            throw std::runtime_error("boundary edge needs a finite increasing domain");
        const double parameter = lo+(hi-lo)*fraction;
        gp_Pnt position;
        gp_Vec tangent;
        curve.D1(parameter,position,tangent);
        tangent.Normalize();
        for (int k=1;k<=3;++k) {
            output[k-1] = position.Coord(k); output[k+2] = tangent.Coord(k);
        }
        gp_Vec normals[2];
        double first_sense = 0;
        for (int side=0;side<2;++side) {
            const auto face = TopoDS::Face(cad->at(row[side+1]));
            bool incident = false;
            TopoDS_Edge occurrence;
            for (TopExp_Explorer it(face,TopAbs_EDGE); it.More(); it.Next())
                if (it.Current().IsSame(edge)) {
                    incident = true;
                    occurrence = TopoDS::Edge(it.Current());
                    if (side == 0) {
                        if (it.Current().Orientation() == TopAbs_FORWARD) first_sense = 1;
                        else if (it.Current().Orientation() == TopAbs_REVERSED) first_sense = -1;
                        else throw std::runtime_error("boundary edge has no wire orientation");
                    }
                    break;
                }
            if (!incident) throw std::runtime_error("boundary face is not incident to the edge");
            // A seam has two p-curves on one face. Reversing the edge selects
            // its other p-curve; ordinary edges use their one curve per face.
            if (side == 1 && row[1] == row[2]) occurrence.Reverse();
            double first,last;
            const auto pcurve = BRep_Tool::CurveOnSurface(occurrence,face,first,last);
            if (pcurve.IsNull() || parameter < first || parameter > last)
                throw std::runtime_error("boundary edge has no matching face parameter curve");
            const auto uv = pcurve->Value(parameter);
            TopLoc_Location location;
            const auto surface = BRep_Tool::Surface(face,location);
            if (surface.IsNull()) throw std::runtime_error("boundary face has no surface");
            gp_Pnt p;
            gp_Vec du,dv;
            surface->D1(uv.X(),uv.Y(),p,du,dv);
            p.Transform(location.Transformation());
            du.Transform(location.Transformation()); dv.Transform(location.Transformation());
            du.Normalize(); dv.Normalize();
            gp_Vec normal = du.Crossed(dv);
            if (normal.Magnitude() < 1e-12) throw std::runtime_error("singular boundary face normal");
            normal.Normalize();
            if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
            else if (face.Orientation() != TopAbs_FORWARD)
                throw std::runtime_error("boundary face has no material orientation");
            const double gap = p.Distance(position);
            if (gap > BRep_Tool::Tolerance(edge)+BRep_Tool::Tolerance(face)+Precision::Confusion())
                throw std::runtime_error("boundary edge is inconsistent with its incident face");
            for (int k=1;k<=3;++k) output[6+3*side+k-1] = normal.Coord(k);
            normals[side] = normal;
            output[12+side] = gap;
        }
        // n_A cross the oriented boundary tangent points into face A. If that
        // direction enters face B's material half-space, the edge is convex.
        const gp_Vec into_first = normals[0].Crossed(tangent)*first_sense;
        output[14] = std::atan2(normals[1].Dot(into_first),normals[0].Dot(normals[1]));
        for (int i=0;i<15;++i) if (!std::isfinite(output[i]))
            throw std::runtime_error("nonfinite boundary query");
        return 0;
    });
}
}
