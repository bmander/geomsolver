# Include at the end of the upstream CMakeLists.txt after defining its libraries.
add_executable(ju_prism_probe "${CMAKE_CURRENT_LIST_DIR}/prism_probe.cpp")
target_compile_features(ju_prism_probe PRIVATE cxx_std_20)
target_link_libraries(ju_prism_probe PRIVATE sweep::sweep lagrange::io)
