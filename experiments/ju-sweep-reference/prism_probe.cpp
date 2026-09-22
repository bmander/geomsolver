// A sharp rectangular-prism input adapter for the public upstream sweep API.
// No changes to the sweep algorithm. Coordinates and motion match run_prisms.py.
#include <sweep/generalized_sweep.h>
#include <lagrange/io/save_mesh.h>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>

std::pair<double, Eigen::RowVector4d> field(Eigen::RowVector4d p, double angle)
{
    const double theta = angle * p[3];
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d velocity(.72, 0., .01);
    const Eigen::Vector3d centre = Eigen::Vector3d(.14, .51, .5) + p[3] * velocity;
    const Eigen::Vector3d relative = p.head<3>().transpose() - centre;
    const Eigen::Vector3d local = rotation.transpose() * relative;
    const Eigen::Vector3d d = local.cwiseAbs() - Eigen::Vector3d(.15, .09, .06);
    const Eigen::Vector3d outside = d.cwiseMax(0.);
    const double length = outside.norm();
    Eigen::Vector3d local_gradient = Eigen::Vector3d::Zero();
    if (length > 0.) {
        for (int i = 0; i < 3; ++i)
            local_gradient[i] = (local[i] >= 0. ? 1. : -1.) * outside[i] / length;
    } else {
        Eigen::Index axis;
        d.maxCoeff(&axis);
        local_gradient[axis] = local[axis] >= 0. ? 1. : -1.;
    }
    const Eigen::Vector3d gradient = rotation * local_gradient;
    const double temporal = -gradient.dot(velocity + Eigen::Vector3d(0., 0., angle).cross(relative));
    return {length + std::min(d.maxCoeff(), 0.),
            Eigen::RowVector4d(gradient[0], gradient[1], gradient[2], temporal)};
}

void check_gradient(double angle)
{
    double maximum = 0.;
    for (int i = 0; i < 100; ++i) {
        Eigen::RowVector4d p(.5 + .43 * std::sin(i * 1.31 + .2),
                            .51 + .22 * std::cos(i * .71 + .3),
                            .5 + .17 * std::sin(i * .37 + .4), (i + .31) / 101.);
        const auto [value, gradient] = field(p, angle);
        if (!std::isfinite(value) || !gradient.allFinite()) throw std::runtime_error("Nonfinite field");
        for (int k = 0; k < 4; ++k) {
            auto plus = p; auto minus = p;
            plus[k] += 1e-6; minus[k] -= 1e-6;
            const double fd = (field(plus, angle).first - field(minus, angle).first) / 2e-6;
            maximum = std::max(maximum, std::abs(fd - gradient[k]));
        }
    }
    std::cout << "Maximum sampled gradient difference: " << maximum << std::endl;
    if (maximum > 1e-5) throw std::runtime_error("Gradient check failed");
}

int main(int argc, char** argv)
{
    if (argc < 2) return 2;
    const double degrees = argc > 2 ? std::stod(argv[2]) : 360.;
    const double epsilon = argc > 3 ? std::stod(argv[3]) : .002;
    const double angle = degrees * std::numbers::pi / 180.;
    check_gradient(angle);
    const std::filesystem::path output(argv[1]);
    std::filesystem::create_directories(output);
    sweep::GridSpec grid;
    sweep::SweepOptions options;
    options.epsilon_env = epsilon;
    options.epsilon_sil = .005;
    options.max_split = 200000;
    std::ofstream(output / "input.json") << "{\"tool\":\"exact box signed distance\",\"size\":[0.3,0.18,0.12],"
        "\"start\":[0.14,0.51,0.5],\"end\":[0.86,0.51,0.51],\"rotation_z_degrees\":" << degrees <<
        ",\"epsilon_env\":" << epsilon << ",\"epsilon_sil\":0.005,\"max_split\":200000}\n";
    auto result = sweep::generalized_sweep([angle](Eigen::RowVector4d p) { return field(p, angle); }, grid, options);
    lagrange::io::save_mesh(output / "envelope.msh", result.envelope);
    lagrange::io::save_mesh(output / "arrangement.msh", result.arrangement);
    lagrange::io::save_mesh(output / "sweep_surface.msh", result.sweep_surface);
}
