#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_ListOfShape.hxx>
#include <Standard_Failure.hxx>
#include <stdexcept>
#include <string>
#include <vector>

// One host session owns every shape. Native operations expose integer handles,
// and no exception may cross the C ABI into Rust.
struct Cad {
    std::vector<TopoDS_Shape> shapes;
    std::string error;
    int put(const TopoDS_Shape& shape) {
        shapes.push_back(shape);
        return static_cast<int>(shapes.size()-1);
    }
    TopoDS_Shape& at(int id) { return shapes.at(static_cast<size_t>(id)); }
};

template<class F> int guarded(Cad* cad,F fn) noexcept {
    try { return fn(); }
    catch (const Standard_Failure& e) { cad->error = e.GetMessageString(); }
    catch (const std::exception& e) { cad->error = e.what(); }
    catch (...) { cad->error = "unknown native CAD exception"; }
    return -1;
}

void validate(TopoDS_Shape& shape);
// Shared non-destructive face partitioning for face and attached-curve tools.
TopoDS_Shape split_face(const TopoDS_Face& face,const TopTools_ListOfShape& tools);
