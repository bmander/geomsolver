#pragma once
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <fstream>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;

inline void write_stl(const Mesh& mesh, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    const std::string header = "Unchecked exact-construction CGAL repair";
    out.write(header.data(),header.size());
    for (size_t i=header.size(); i<80; ++i) out.put(' ');
    const uint32_t count = mesh.number_of_faces();
    out.write(reinterpret_cast<const char*>(&count),4);
    for (auto f : mesh.faces()) {
        const float normal[3] = {};
        out.write(reinterpret_cast<const char*>(normal),12);
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f),mesh)) {
            const auto& p = mesh.point(v);
            const float point[3] = {float(CGAL::to_double(p.x())),float(CGAL::to_double(p.y())),
                                    float(CGAL::to_double(p.z()))};
            out.write(reinterpret_cast<const char*>(point),12);
        }
        const uint16_t attribute = 0;
        out.write(reinterpret_cast<const char*>(&attribute),2);
    }
    if (!out) throw std::runtime_error("STL write failed");
}
