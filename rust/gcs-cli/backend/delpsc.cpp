// Delaunay refinement of a material field's boundary with protected 1D features: CGAL's Mesh_3,
// the implementation of Cheng, Dey and Ramos's refinement for piecewise-smooth complexes (with
// Dey and Levine's protecting balls). An experiment behind the `cgal` feature: CGAL's Mesh_3 is
// GPL-3.0, so nothing here ships in the default build.
//
// The domain is labelled by the sign of a caller's field (negative is material), which Mesh_3
// asks at points and bisects along segments; the features are polylines the caller supplies.
// The boundary facets of the refined complex are written as a binary STL in millimetres.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_domain_with_polyline_features_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/facets_in_complex_3_to_triangle_mesh.h>
#include <CGAL/make_mesh_3.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <string>
#include <vector>

namespace {
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_3;
using Base = CGAL::Labeled_mesh_domain_3<K>;
using Domain = CGAL::Mesh_domain_with_polyline_features_3<Base>;
using Tr = CGAL::Mesh_triangulation_3<Domain,CGAL::Default,CGAL::Sequential_tag>::type;
using C3t3 = CGAL::Mesh_complex_3_in_triangulation_3<Tr,Domain::Corner_index,Domain::Curve_index>;
using Criteria = CGAL::Mesh_criteria_3<Tr>;
namespace params = CGAL::parameters;

void fail(char* error,int capacity,const std::string& message) {
    if (!error || capacity <= 0) return;
    std::strncpy(error,message.c_str(),capacity-1);
    error[capacity-1] = 0;
}
}

extern "C" {
// `sizes`: facet size, facet distance, facet angle (degrees), feature edge size, and the
// relative error bound of the bisection, all in model units but the angle. `curves` polylines
// of `counts[i]` points each follow in `points` (x, y, z). Returns the triangle count written,
// or -1 with a message.
int solvent_cgal_mesh(double (*field)(void*,const double*),void* context,const double* center,double radius,
    const double* points,const int* counts,int curves,const double* sizes,double scale,const char* path,
    char* error,int capacity) noexcept {
    try {
        // Labelled by side: subdomain 1 is material, 0 the outside. (The domain's function is a
        // labelling, not a signed field: a value handed to it is read as a subdomain index.)
        const std::function<int(const Point&)> function = [&](const Point& p) {
            const double q[3] = {p.x(),p.y(),p.z()};
            return field(context,q) < 0 ? 1 : 0;
        };
        Domain domain(params::function(function)
            .bounding_object(K::Sphere_3(Point(center[0],center[1],center[2]),radius*radius))
            .relative_error_bound(sizes[4]));
        std::vector<std::vector<Point>> polylines;
        for (int c=0,k=0;c<curves;++c) {
            std::vector<Point> line;
            for (int i=0;i<counts[c];++i,++k) line.emplace_back(points[3*k],points[3*k+1],points[3*k+2]);
            polylines.push_back(std::move(line));
        }
        if (!polylines.empty()) domain.add_features(polylines.begin(),polylines.end());
        Criteria criteria(params::edge_size(sizes[3]).edge_min_size(0.1*sizes[3]).facet_size(sizes[0]).facet_distance(sizes[1])
            .facet_angle(sizes[2]).cell_radius_edge_ratio(3.));
        C3t3 complex = CGAL::make_mesh_3<C3t3>(domain,criteria,params::no_perturb().no_exude().manifold());
        std::vector<Point> vertices;
        std::vector<std::array<std::size_t,3>> triangles;
        std::vector<C3t3::Surface_patch_index> patches;
        CGAL::SMDS_3::internal::facets_in_complex_3_to_triangle_soup(complex,vertices,triangles,patches);
        // Orient outward by the signed volume the triangles enclose.
        double volume = 0.;
        for (const auto& t : triangles) {
            const auto &a = vertices[t[0]],&b = vertices[t[1]],&c = vertices[t[2]];
            volume += (a.x()*(b.y()*c.z()-b.z()*c.y())-a.y()*(b.x()*c.z()-b.z()*c.x())+a.z()*(b.x()*c.y()-b.y()*c.x()))/6.;
        }
        FILE* out = std::fopen(path,"wb");
        if (!out) { fail(error,capacity,std::string("cannot write ")+path); return -1; }
        char header[80] = {0};
        std::snprintf(header,80,"solvent cgal mesh_3");
        std::fwrite(header,1,80,out);
        const std::uint32_t count = static_cast<std::uint32_t>(triangles.size());
        std::fwrite(&count,4,1,out);
        for (auto t : triangles) {
            if (volume < 0) std::swap(t[1],t[2]);
            float record[12] = {0};
            for (int v=0;v<3;++v) for (int i=0;i<3;++i)
                record[3+3*v+i] = static_cast<float>(vertices[t[v]][i]*scale);
            std::fwrite(record,4,12,out);
            const std::uint16_t attribute = 0;
            std::fwrite(&attribute,2,1,out);
        }
        std::fclose(out);
        return static_cast<int>(count);
    } catch (const std::exception& e) {
        fail(error,capacity,e.what());
        return -1;
    } catch (...) {
        fail(error,capacity,"CGAL meshing failed");
        return -1;
    }
}
}
