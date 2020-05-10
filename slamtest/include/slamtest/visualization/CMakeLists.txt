add_subdirectory( opengl )

add_definitions( -DPYTHON_PLOT_PATH="${CMAKE_SOURCE_DIR}/src/visualization/" )
set( VIS_HEADERS vis.h plot.py gvis.h gnuplot_i.h )
set( VIS_SOURCE vis.cpp gvis.cpp gnuplot_i.cpp )
add_library( visualization ${VIS_SOURCE} ${VIS_HEADERS} )
target_link_libraries( visualization ${PYTHON_LIBRARIES} )