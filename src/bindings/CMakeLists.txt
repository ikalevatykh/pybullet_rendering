cmake_minimum_required(VERSION 3.1)

find_package(PythonLibs REQUIRED)

set(MODULE_NAME "bindings")

pybind11_add_module(bindings bindings.cpp)
target_link_libraries(bindings
  PUBLIC
    plugin
)
