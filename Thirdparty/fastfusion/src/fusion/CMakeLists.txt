


add_library(geometryfusion_aos STATIC
geometryfusion_aos.cpp
treeandbrick.cpp
treeandbrick_indexed.cpp
treeandbrick_incremental.cpp
treeandbrick_incremental_recursive.cpp
mesh.cpp
#meshcelltraversal.cpp
mesh_interleaved.cpp
mesh_interleaved_meshcell.cpp
)

target_link_libraries(geometryfusion_aos
${EXTERNAL_LIBS})




add_library(geometryfusion_mipmap_cpu STATIC
geometryfusion_mipmap_cpu.cpp
loopclosure.cpp
)

target_link_libraries(geometryfusion_mipmap_cpu
${EXTERNAL_LIBS})


