// Exact-construction repair trial; independently audit the rounded STL afterward.
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/boost/graph/helpers.h>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;

static void write_stl(const Mesh& mesh, const std::string& path) {
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

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::invalid_argument("usage: solvent-cgal-repair INPUT.stl OUTPUT.stl");
        if (std::filesystem::weakly_canonical(argv[1]) == std::filesystem::weakly_canonical(argv[2]))
            throw std::invalid_argument("source must be retained; choose another output path");
        Mesh mesh;
        if (!CGAL::IO::read_polygon_mesh(argv[1],mesh) || !CGAL::is_triangle_mesh(mesh))
            throw std::runtime_error("input is not a readable triangle mesh");
        const size_t before = mesh.number_of_faces();
        const bool input_closed = CGAL::is_closed(mesh);
        const bool input_intersects = PMP::does_self_intersect(mesh);
        auto started = std::chrono::steady_clock::now();
        const bool success = PMP::experimental::autorefine_and_remove_self_intersections(mesh);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        const bool output_intersects = PMP::does_self_intersect(mesh);
        if (!CGAL::is_triangle_mesh(mesh)) throw std::runtime_error("repair produced non-triangular faces");
        write_stl(mesh,argv[2]);
        std::ofstream report(std::string(argv[2])+".repair.json");
        report << std::boolalpha << std::setprecision(17)
               << "{\"backend\":\"CGAL-6.1.2-exact-autorefine-remove\",\"success\":" << success
               << ",\"input_triangles\":" << before << ",\"input_closed\":" << input_closed
               << ",\"input_intersects\":" << input_intersects
               << ",\"output_triangles\":" << mesh.number_of_faces()
               << ",\"output_closed\":" << CGAL::is_closed(mesh)
               << ",\"output_intersects_exact\":" << output_intersects
               << ",\"repair_seconds\":" << seconds
               << ",\"scope\":\"Native exact checks only; rounded STL requires independent audit.\"}\n";
        if (!report) throw std::runtime_error("report write failed");
        std::cout << "success=" << success << ", faces=" << mesh.number_of_faces()
                  << ", closed=" << CGAL::is_closed(mesh) << ", intersects=" << output_intersects
                  << ", repair_ms=" << seconds*1000 << '\n';
        return success ? 0 : 1;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
