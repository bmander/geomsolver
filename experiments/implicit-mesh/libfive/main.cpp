// Isolated native baseline. No Solvent field adapter or mesh repair.
#include "fixtures.hpp"
#include <libfive.h>
#include <libfive/eval/eval_array.hpp>
#include <libfive/render/brep/mesh.hpp>
#include <libfive/render/brep/settings.hpp>
#include <libfive/render/brep/region.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now()-start).count();
}
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--sample") {
            std::string name;
            float x, y, z;
            while (std::cin >> name >> x >> y >> z) {
                libfive::ArrayEvaluator evaluator(fixture(name).first);
                std::cout << std::setprecision(9) << evaluator.value(Eigen::Vector3f(x,y,z)) << '\n';
            }
            return 0;
        }
        if (argc != 5) throw std::invalid_argument("usage: solvent-libfive-bench CASE|all dc|simplex|hybrid DEPTH OUTPUT_DIRECTORY");
        const std::string selection = argv[1], algorithm = argv[2];
        const int depth = std::stoi(argv[3]);
        if (depth < 2 || depth > 9) throw std::invalid_argument("depth must be 2..9");
        libfive::BRepSettings settings;
        settings.workers = 1;
        if (algorithm == "dc") settings.alg = libfive::DUAL_CONTOURING;
        else if (algorithm == "simplex") settings.alg = libfive::ISO_SIMPLEX;
        else if (algorithm == "hybrid") settings.alg = libfive::HYBRID;
        else throw std::invalid_argument("unknown meshing algorithm");
        if (selection != "all" && std::find(names.begin(),names.end(),selection) == names.end())
            throw std::invalid_argument("unknown fixture");
        std::filesystem::create_directories(argv[4]);
        for (const auto& name : selection == "all" ? names : std::vector<std::string>{selection}) {
            auto start = Clock::now();
            auto [tree, extent] = fixture(name);
            const double setup = elapsed(start);
            settings.min_feature = 2*extent / (1u << depth);
            libfive::Region<3> region(Eigen::Vector3d::Constant(-extent),Eigen::Vector3d::Constant(extent));
            start = Clock::now();
            auto mesh = libfive::Mesh::render(tree,region,settings);
            const double seconds = elapsed(start);
            if (!mesh) throw std::runtime_error("libfive meshing failed or cancelled");
            const auto stem = std::filesystem::path(argv[4]) /
                (name+"-"+algorithm+"-depth"+std::to_string(depth));
            if (!mesh->saveSTL(stem.string()+".stl")) throw std::runtime_error("STL write failed");
            std::ofstream out(stem.string()+".json");
            out << std::setprecision(17) << "{\"case\":\"" << name << "\",\"backend\":\"libfive-"
                << libfive_git_revision() << "-" << algorithm << "\",\"depth\":" << depth
                << ",\"min_feature\":" << settings.min_feature << ",\"max_err\":" << settings.max_err
                << ",\"extent\":" << extent << ",\"threads\":1,\"point_queries\":null,\"setup_seconds\":"
                << setup << ",\"extraction_seconds\":" << seconds
                << ",\"status\":\"unchecked_expression_baseline\",\"vertices\":[";
            for (size_t i=0; i<mesh->verts.size(); ++i) {
                const auto& v = mesh->verts[i];
                out << (i ? ",[" : "[") << v.x() << ',' << v.y() << ',' << v.z() << ']';
            }
            out << "],\"triangles\":[";
            for (size_t i=0; i<mesh->branes.size(); ++i) {
                const auto& t = mesh->branes[i];
                out << (i ? ",[" : "[") << t.x() << ',' << t.y() << ',' << t.z() << ']';
            }
            out << "]}\n";
            if (!out) throw std::runtime_error("JSON write failed");
            std::cout << name << ' ' << algorithm << " depth " << depth << ": " << mesh->branes.size()
                      << " triangles, " << seconds*1000 << " ms\n" << std::flush;
        }
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
