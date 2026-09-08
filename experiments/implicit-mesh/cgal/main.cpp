// Exact-construction repair trial; independently audit the rounded STL afterward.
#include "mesh_io.hpp"
#include <CGAL/boost/graph/IO/polygon_mesh_io.h>
#if SOLVENT_CGAL_REPAIR_MODE
#include <CGAL/Polygon_mesh_processing/repair_self_intersections.h>
#else
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#endif
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/boost/graph/helpers.h>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <cmath>

namespace PMP = CGAL::Polygon_mesh_processing;

int main(int argc, char** argv) {
    try {
#if SOLVENT_CGAL_REPAIR_MODE
        if (argc != 4) throw std::invalid_argument("usage: solvent-cgal-local INPUT.stl OUTPUT.stl EPSILON");
        size_t parsed = 0;
        const double epsilon = std::stod(argv[3], &parsed);
        if (parsed != std::string(argv[3]).size() || !std::isfinite(epsilon) || epsilon <= 0)
            throw std::invalid_argument("epsilon must be finite and positive");
        const bool round_cleanup = false;
#else
        if (argc != 3 && argc != 4)
            throw std::invalid_argument("usage: solvent-cgal-repair INPUT.stl OUTPUT.stl [--round-cleanup]");
        const bool round_cleanup = argc == 4 && std::string(argv[3]) == "--round-cleanup";
        if (argc == 4 && !round_cleanup) throw std::invalid_argument("unknown cleanup option");
#endif
        if (std::filesystem::weakly_canonical(argv[1]) == std::filesystem::weakly_canonical(argv[2]))
            throw std::invalid_argument("source must be retained; choose another output path");
        Mesh mesh;
        if (!CGAL::IO::read_polygon_mesh(argv[1],mesh) || !CGAL::is_triangle_mesh(mesh))
            throw std::runtime_error("input is not a readable triangle mesh");
        const size_t before = mesh.number_of_faces();
        const bool input_closed = CGAL::is_closed(mesh);
        const bool input_intersects = PMP::does_self_intersect(mesh);
        auto started = std::chrono::steady_clock::now();
#if SOLVENT_CGAL_REPAIR_MODE
        bool success = PMP::experimental::remove_self_intersections(mesh,
            CGAL::parameters::polyhedral_envelope_epsilon(epsilon).preserve_genus(true));
#else
        bool success = PMP::experimental::autorefine_and_remove_self_intersections(mesh);
#endif
        bool rounding_fixed_point = false;
        if (round_cleanup) {
            for (auto v : mesh.vertices()) {
                const auto p = mesh.point(v);
                mesh.point(v) = Kernel::Point_3(float(CGAL::to_double(p.x())),
                    float(CGAL::to_double(p.y())),float(CGAL::to_double(p.z())));
            }
            success = PMP::remove_degenerate_faces(mesh) && success;
            rounding_fixed_point = true;
            for (auto v : mesh.vertices()) {
                const auto& p = mesh.point(v);
                const Kernel::Point_3 rounded(float(CGAL::to_double(p.x())),
                    float(CGAL::to_double(p.y())),float(CGAL::to_double(p.z())));
                rounding_fixed_point = rounding_fixed_point && p == rounded;
            }
            success = success && rounding_fixed_point;
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        const bool output_intersects = PMP::does_self_intersect(mesh);
        const bool library_success = success;
        success = success && !output_intersects && (!input_closed || CGAL::is_closed(mesh))
                  && (before == 0 || mesh.number_of_faces() > 0);
        if (!CGAL::is_triangle_mesh(mesh)) throw std::runtime_error("repair produced non-triangular faces");
        write_stl(mesh,argv[2]);
        std::ofstream report(std::string(argv[2])+".repair.json");
        report << std::boolalpha << std::setprecision(17)
               << "{\"backend\":\"CGAL-6.1.2-"
#if SOLVENT_CGAL_REPAIR_MODE
               << "local-envelope-repair\",\"proximity_epsilon\":" << epsilon
#else
               << "exact-autorefine-remove\""
#endif
               << ",\"success\":" << success << ",\"library_success\":" << library_success
               << ",\"input_triangles\":" << before << ",\"input_closed\":" << input_closed
               << ",\"input_intersects\":" << input_intersects
               << ",\"output_triangles\":" << mesh.number_of_faces()
               << ",\"output_closed\":" << CGAL::is_closed(mesh)
               << ",\"output_intersects_exact\":" << output_intersects
               << ",\"round_cleanup\":" << round_cleanup
               << ",\"rounding_fixed_point\":" << rounding_fixed_point
               << ",\"repair_seconds\":" << seconds
               << ",\"scope\":\"Native exact checks only; rounded STL requires independent audit.\"}\n";
        if (!report) throw std::runtime_error("report write failed");
        std::cout << "success=" << success << ", faces=" << mesh.number_of_faces()
                  << ", closed=" << CGAL::is_closed(mesh) << ", intersects=" << output_intersects
                  << ", repair_ms=" << seconds*1000 << '\n';
        return success ? 0 : 1;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
