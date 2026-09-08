#!/bin/sh
set -eu
if [ "$#" -lt 4 ] || [ "$#" -gt 5 ]; then
    echo "usage: sh build.sh CGAL_ROOT BOOST_ROOT GMP_PREFIX OUTPUT_BINARY [repair|degenerate]" >&2
    exit 2
fi
case "${5:-repair}" in
    repair) source_name=main.cpp ;;
    degenerate) source_name=degenerate.cpp ;;
    *) echo "unknown operation" >&2; exit 2 ;;
esac
# Header-only CGAL; avoid requiring a system-wide Boost CMake installation.
"${CXX:-c++}" -std=c++17 -O1 -frounding-math \
    -I"$1/include" -I"$2" -I"$3/include" \
    "$(dirname "$0")/$source_name" -L"$3/lib" -lgmp -lmpfr -o "$4"
