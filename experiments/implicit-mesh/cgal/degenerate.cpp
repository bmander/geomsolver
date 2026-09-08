// Preserve indexed connectivity while collapsing exactly degenerate elements.
#include "mesh_io.hpp"
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace PMP = CGAL::Polygon_mesh_processing;

static Mesh read_indexed(const char* path) {
    std::ifstream in(path);
    size_t vertex_count, face_count, ignored;
    std::string tag;
    if (!(in >> tag >> vertex_count >> face_count >> ignored) || tag != "OFF")
        throw std::runtime_error("invalid indexed mesh header");
    Mesh mesh;
    std::vector<Mesh::Vertex_index> vertices;
    for (size_t i=0; i<vertex_count; ++i) {
        double x, y, z;
        if (!(in >> x >> y >> z) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            throw std::runtime_error("invalid indexed coordinate");
        // Construct exact values from binary64, rather than interpreting decimals as rationals.
        vertices.push_back(mesh.add_vertex(Kernel::Point_3(x,y,z)));
    }
    for (size_t i=0; i<face_count; ++i) {
        size_t n, a, b, c;
        if (!(in >> n >> a >> b >> c) || n != 3 || a>=vertex_count || b>=vertex_count || c>=vertex_count)
            throw std::runtime_error("invalid indexed triangle");
        if (mesh.add_face(vertices[a],vertices[b],vertices[c]) == Mesh::null_face())
            throw std::runtime_error("input indexed topology is not a valid polygon mesh");
    }
    return mesh;
}
static size_t degenerates(const Mesh& mesh) {
    size_t count = 0;
    for (auto face : mesh.faces()) count += PMP::is_degenerate_triangle_face(face,mesh);
    return count;
}
int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::invalid_argument("usage: solvent-cgal-degenerate INPUT.off OUTPUT.stl");
        if (std::filesystem::weakly_canonical(argv[1]) == std::filesystem::weakly_canonical(argv[2]))
            throw std::invalid_argument("source must be retained");
        Mesh mesh = read_indexed(argv[1]);
        const size_t before = mesh.number_of_faces(), degenerate_before = degenerates(mesh);
        const bool closed_before = CGAL::is_closed(mesh);
        auto start = std::chrono::steady_clock::now();
        std::vector<bool> library_returns;
        size_t remaining = degenerate_before;
        for (unsigned pass=0; remaining && pass<4; ++pass) {
            const size_t previous = remaining, previous_faces = mesh.number_of_faces();
            library_returns.push_back(PMP::remove_degenerate_faces(mesh));
            remaining = degenerates(mesh);
            if (remaining == previous && mesh.number_of_faces() == previous_faces) break;
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        const size_t degenerate_after = degenerates(mesh);
        const bool success = degenerate_after == 0 && (!closed_before || CGAL::is_closed(mesh));
        if (!CGAL::is_triangle_mesh(mesh)) throw std::runtime_error("cleanup produced non-triangular faces");
        write_stl(mesh,argv[2]);
        std::ofstream out(std::string(argv[2])+".cleanup.json");
        out << std::boolalpha << std::setprecision(17)
            << "{\"backend\":\"CGAL-6.1.2-remove-degenerate-faces\",\"success\":" << success
            << ",\"input_faces\":" << before << ",\"input_degenerate\":" << degenerate_before
            << ",\"input_closed\":" << closed_before << ",\"output_faces\":" << mesh.number_of_faces()
            << ",\"output_degenerate\":" << degenerate_after
            << ",\"output_closed\":" << CGAL::is_closed(mesh) << ",\"cleanup_seconds\":" << seconds
            << ",\"passes\":" << library_returns.size() << ",\"library_returns\":[";
        for (size_t i=0; i<library_returns.size(); ++i) out << (i ? "," : "") << library_returns[i];
        out << "]}\n";
        if (!out) throw std::runtime_error("cleanup report write failed");
        std::cout << "success=" << success << " degenerate=" << degenerate_before << "->" << degenerate_after
                  << " faces=" << mesh.number_of_faces() << " closed=" << CGAL::is_closed(mesh)
                  << " cleanup_ms=" << seconds*1000 << '\n';
        return success ? 0 : 1;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
