include_directories(${CMAKE_SOURCE_DIR})

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_QTOPENGL_FOUND)
  add_subdirectory(id_loop_functions)
endif(ARGOS_QTOPENGL_FOUND)

