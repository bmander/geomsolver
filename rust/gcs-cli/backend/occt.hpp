#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
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
struct Cad {
    std::vector<TopoDS_Shape> shapes;
    std::vector<char> valid;
    std::vector<double> volumes;
    // Volumes `validate` measured of solids inside a stored shape (a partition's cells),
    // for the handles those solids are given when they are listed.
    TopTools_DataMapOfShapeReal measured;
    std::string error;
    int put(const TopoDS_Shape& shape) {
        shapes.push_back(shape);
        valid.push_back(0);
        volumes.push_back(std::numeric_limits<double>::quiet_NaN());
        return static_cast<int>(shapes.size()-1);
    }
    TopoDS_Shape& at(int id) { return shapes.at(static_cast<size_t>(id)); }
    // Validate a stored shape unless it already has been.
    void validated(int id);
    // The stored shape's volume, measured once.
    double volume_of(int id);
};

template<class F> int guarded(Cad* cad,F fn) noexcept {
    try { return fn(); }
    catch (const Standard_Failure& e) { cad->error = e.GetMessageString(); }
    catch (const std::exception& e) { cad->error = e.what(); }
    catch (...) { cad->error = "unknown native CAD exception"; }
    return -1;
}

// Throws unless the shape is a valid closed solid of positive volume; returns that volume.
// Each solid's volume is recorded in `record` when one is given.
double validate(TopoDS_Shape& shape,TopTools_DataMapOfShapeReal* record = nullptr);
