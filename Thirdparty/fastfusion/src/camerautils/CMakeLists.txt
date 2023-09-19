set(camerautils_SOURCES camerautils.cpp)

add_library(camerautils ${camerautils_SOURCES})
target_link_libraries(camerautils
${EXTERNAL_LIBS})
