#pragma once
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <fstream>
#include <iomanip>
#include <map>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;

// Diagnostic interchange retains topology; exact constructed coordinates round to binary64.
inline void write_indexed(const Mesh& mesh, const std::string& path) {
    std::ofstream out(path);
    std::map<Mesh::Vertex_index, size_t> indices;
    out << std::setprecision(17) << "{\"vertices\":[";
    for (auto v : mesh.vertices()) {
        const size_t i = indices.size();
        indices.emplace(v,i);
        const auto& p = mesh.point(v);
        out << (i ? ",[" : "[") << CGAL::to_double(p.x()) << ','
            << CGAL::to_double(p.y()) << ',' << CGAL::to_double(p.z()) << ']';
    }
    out << "],\"triangles\":[";
    size_t face_index = 0;
    for (auto f : mesh.faces()) {
        out << (face_index++ ? ",[" : "[");
        size_t corner = 0;
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f),mesh))
            out << (corner++ ? "," : "") << indices.at(v);
        out << ']';
    }
    out << "]}\n";
    if (!out) throw std::runtime_error("indexed mesh write failed");
}

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
