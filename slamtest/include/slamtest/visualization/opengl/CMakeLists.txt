add_definitions( -DGL_SHADER_PATH="${CMAKE_SOURCE_DIR}/src/visualization/opengl/shaders/" )

set( GL_VIS_HEADERS glvis.h opengl_utils.h )
set( GL_VIS_SOURCE glvis.cpp opengl_util.cpp )

add_library( gl_visualization ${GL_VIS_SOURCE} ${GL_VIS_HEADERS} )	
target_link_libraries( gl_visualization glad glfw ${CMAKE_THREAD_LIBS_INIT} )