#!/bin/sh
set -eu
if [ "$#" -lt 4 ] || [ "$#" -gt 5 ]; then
    echo "usage: sh build.sh CGAL_ROOT BOOST_ROOT GMP_PREFIX OUTPUT_BINARY [repair|degenerate|local]" >&2
    exit 2
fi
repair_mode=0
case "${5:-repair}" in
    repair) source_name=main.cpp ;;
    degenerate) source_name=degenerate.cpp ;;
    local) source_name=main.cpp; repair_mode=1 ;;
    *) echo "unknown operation" >&2; exit 2 ;;
esac
# Header-only CGAL; avoid requiring a system-wide Boost CMake installation.
"${CXX:-c++}" -std=c++17 -O1 -frounding-math -DSOLVENT_CGAL_REPAIR_MODE="$repair_mode" \
    -I"$1/include" -I"$2" -I"$3/include" \
    "$(dirname "$0")/$source_name" -L"$3/lib" -lgmp -lmpfr -o "$4"
