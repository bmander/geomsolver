#!/bin/sh
set -eu
if [ "$#" -ne 4 ]; then
    echo "usage: sh build.sh CGAL_ROOT BOOST_ROOT GMP_PREFIX OUTPUT_BINARY" >&2
    exit 2
fi
# Header-only CGAL; avoid requiring a system-wide Boost CMake installation.
"${CXX:-c++}" -std=c++17 -O1 -frounding-math \
    -I"$1/include" -I"$2" -I"$3/include" \
    "$(dirname "$0")/main.cpp" -L"$3/lib" -lgmp -lmpfr -o "$4"
