// Expression equivalents of the other mesher fixtures, without supplied features.
#include "fixtures.hpp"
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <stdexcept>

using libfive::Tree;
using V = Eigen::Vector3d;
using namespace libfive;

const std::vector<std::string> names = {"sphere", "cube", "rotated_cube", "torus",
    "spiky_tetrahedron", "rotated_tetrahedron", "thin_plate", "disconnected", "zero_only"};

static V rotate(const V& p, double angle) {
    return Eigen::AngleAxisd(angle, V(1,2,3).normalized()) * p;
}
static Tree linear(const V& n) {
    return Tree::X()*n.x() + Tree::Y()*n.y() + Tree::Z()*n.z();
}
static Tree ball(const V& center, double radius) {
    return sqrt(square(Tree::X()-center.x()) + square(Tree::Y()-center.y()) +
                square(Tree::Z()-center.z())) - radius;
}
static Tree box(double angle, const V& half) {
    return max(max(abs(linear(rotate(V::UnitX(), angle)))-half.x(),
                   abs(linear(rotate(V::UnitY(), angle)))-half.y()),
                   abs(linear(rotate(V::UnitZ(), angle)))-half.z());
}
static Tree tetrahedron(double angle) {
    std::array<V,4> p = {V(0,0,1.5), V(.06,0,-.5),
                        V(-.03,.03*std::sqrt(3.),-.5), V(-.03,-.03*std::sqrt(3.),-.5)};
    V center = V::Zero();
    for (auto& point : p) { point = rotate(point, angle); center += point/4; }
    std::vector<Tree> planes;
    for (const auto& t : std::array<std::array<int,3>,4>{{{0,1,2},{0,2,3},{0,3,1},{1,3,2}}}) {
        const V a = p[t[0]];
        V n = (p[t[1]]-a).cross(p[t[2]]-a).normalized();
        if (n.dot(center-a) > 0) n = -n;
        planes.push_back(linear(n)-n.dot(a));
    }
    return max(max(planes[0],planes[1]),max(planes[2],planes[3]));
}
std::pair<Tree,double> fixture(const std::string& name) {
    if (name == "sphere") return {ball(V::Zero(),1),1.25};
    if (name == "cube") return {box(0,V::Ones()),1.25};
    if (name == "rotated_cube") return {box(.47,V::Ones()),1.75};
    if (name == "thin_plate") return {box(.47,V(.02,.7,.7)),1.25};
    if (name == "spiky_tetrahedron") return {tetrahedron(0),1.75};
    if (name == "rotated_tetrahedron") return {tetrahedron(.47),1.75};
    if (name == "disconnected") return {min(ball(V(-.4,0,0),.3),ball(V(.53,.11,.07),.035)),1};
    if (name == "zero_only") { auto a = ball(V::Zero(),1); return {max(a,-a),1.25}; }
    if (name == "torus") {
        auto r = sqrt(square(Tree::X())+square(Tree::Y()))-2;
        return {sqrt(square(r)+square(Tree::Z()))-.6,3};
    }
    throw std::invalid_argument("unknown fixture");
}
