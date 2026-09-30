#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax1.hxx>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_DataMapOfShapeReal.hxx>
#include <Standard_Failure.hxx>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// One host session owns every shape. Native operations expose integer handles,
// and no exception may cross the C ABI into Rust.
// A stored shape never changes, so what is learned of it is kept beside it: that it passed
// `validate` (which may reorient it in place, once) and its volume, NaN until measured. A
// member's solid is validated and measured once, not again by each output written from it.
// A session may be called from several threads at once (a body's sweeps are built side by side):
// its tables are guarded, a handle indexes a deque (whose elements stay where they are as it
// grows), and each thread keeps its own last error. Threads share a shape only to read it: an
// operation that would change its input (a Boolean's tolerances) is run non-destructively.
struct Cad {
    int put(const TopoDS_Shape& shape,bool checked = false,double volume = std::numeric_limits<double>::quiet_NaN()) {
        const std::lock_guard<std::mutex> hold(lock);
        shapes.push_back(shape);
        valid.push_back(checked);
        volumes.push_back(volume);
        return static_cast<int>(shapes.size()-1);
    }
    TopoDS_Shape& at(int id) {
        const std::lock_guard<std::mutex> hold(lock);
        return shapes.at(static_cast<size_t>(id));
    }
    // Validate a stored shape unless it already has been.
    void validated(int id);
    // The stored shape's volume, measured once.
    double volume_of(int id);
    // The stored shape's volume where it is known already (NaN where not).
    double known_volume(int id) { const std::lock_guard<std::mutex> hold(lock); return volumes.at(static_cast<size_t>(id)); }
    // Volumes `validate` measured of solids inside a stored shape (a partition's cells), kept for
    // the handles those solids are given when they are listed; and one read back.
    void record(const TopTools_DataMapOfShapeReal& volumes);
    bool recorded(const TopoDS_Shape& solid,double& volume);
    // The solids `solvent_cad_pattern` made: the axis they are turned about and how many copies.
    void set_pattern(int id,const gp_Ax1& axis,int copies) { const std::lock_guard<std::mutex> hold(lock); patterns[id] = {axis,copies}; }
    bool pattern(int id,gp_Ax1& axis,int& copies) {
        const std::lock_guard<std::mutex> hold(lock);
        const auto it = patterns.find(id);
        if (it == patterns.end()) return false;
        axis = it->second.first; copies = it->second.second;
        return true;
    }
    // A pattern's union made and not yet checked (`solvent_cad_pattern`), what its check
    // (`solvent_cad_pattern_check`) needs kept under its handle until then (cells.cpp).
    struct Pending;
    void set_pending(int id,std::shared_ptr<Pending> pending) { const std::lock_guard<std::mutex> hold(lock); pendings[id] = std::move(pending); }
    std::shared_ptr<Pending> take_pending(int id) {
        const std::lock_guard<std::mutex> hold(lock);
        const auto it = pendings.find(id);
        if (it == pendings.end()) return nullptr;
        auto pending = it->second;
        pendings.erase(it);
        return pending;
    }
    // A stored shape found valid by a check of its own, with its volume.
    void set_checked(int id,double volume) {
        const std::lock_guard<std::mutex> hold(lock);
        valid.at(static_cast<size_t>(id)) = 1;
        volumes.at(static_cast<size_t>(id)) = volume;
    }
private:
    std::map<int,std::shared_ptr<Pending>> pendings;
    std::mutex lock;
    std::deque<TopoDS_Shape> shapes;
    std::deque<char> valid;
    std::deque<double> volumes;
    TopTools_DataMapOfShapeReal measured;
    std::map<int,std::pair<gp_Ax1,int>> patterns;
};

// The calling thread's last native error, which `guarded` sets.
std::string& last_error();

template<class F> int guarded(Cad* cad,F fn) noexcept {
    try { return fn(); }
    catch (const Standard_Failure& e) { last_error() = e.GetMessageString(); }
    catch (const std::exception& e) { last_error() = e.what(); }
    catch (...) { last_error() = "unknown native CAD exception"; }
    return -1;
}

// Throws unless the shape is a valid closed solid of positive volume; returns that volume.
// Each solid's volume is recorded in `record` when one is given. `checked`: the caller has just
// run the kernel's checker over this very shape, and it passed. `known`: the volume of the one
// solid, which the caller has measured another way (NaN: measure it).
double validate(TopoDS_Shape& shape,TopTools_DataMapOfShapeReal* record = nullptr,bool checked = false,
    double known = std::numeric_limits<double>::quiet_NaN());

// A solid's volume, adaptive to 1e-9 relative on each face, its faces integrated on every core.
double volume(const TopoDS_Shape& shape);
// The faces' flux about `origin`: the volume they bound when they close (occt.cpp).
double flux(const std::vector<TopoDS_Face>& faces,const gp_Pnt& origin);
// A shape's area, its faces measured on every core (occt.cpp).
double area(const TopoDS_Shape& shape);
// The kernel's checker.
bool valid(const TopoDS_Shape& shape);
// Whether Booleans run their intersections on every core (occt.cpp).
bool parallel_booleans();
// The same over one closed solid, its faces on every core (occt.cpp).
bool valid_solid(const TopoDS_Shape& shape);
// Its closure and orientation alone, where its faces were checked already (occt.cpp).
bool closed_and_oriented(const TopoDS_Shape& shape);

// Raise `worst` to the chordal sag a meshed face has, and `at` to where (occt.cpp).
void face_sag(const TopoDS_Face& face,double& worst,gp_Pnt& at);

// What the kernel's checker finds wrong with a shape, for a message.
std::string invalidity(const TopoDS_Shape& shape);
