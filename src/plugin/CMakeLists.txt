# bullet
file(GLOB_RECURSE plugin_SOURCES "*.cpp")
add_library(plugin STATIC ${plugin_SOURCES})

target_compile_definitions(plugin
  PRIVATE
    # pybullet uses double precision by default
    BT_USE_DOUBLE_PRECISION=1
)

set(BULLET_SOURCE_DIR "${BULLET_ROOT_PATH}/src/")
set(BULLET_EXAMPLES_DIR "${BULLET_ROOT_PATH}/examples/")

target_include_directories(plugin
  PRIVATE
    "${BULLET_SOURCE_DIR}"
    "${BULLET_EXAMPLES_DIR}"
)

target_link_libraries(plugin
  PUBLIC
    render
    scene
)
