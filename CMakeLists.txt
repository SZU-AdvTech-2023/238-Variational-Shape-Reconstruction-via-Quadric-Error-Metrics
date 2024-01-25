set(CGAL_Qt5_FOUND FALSE)

# add vendor
add_subdirectory(vendor)

# src groups
set(VSR_SRC_GROUPS)
set(VSR_HEADER_GROUPS)
function(add_directory dir)
    aux_source_directory("src/${dir}" DIR_SRC)
    source_group("Source Files\\${dir}" FILES ${DIR_SRC})
    file(GLOB DIR_HEADERS include/${dir}/*.h)
    source_group("Header Files\\${dir}" FILES ${DIR_HEADERS})
    
    list(APPEND VSR_SRC_GROUPS ${DIR_SRC})
    set(VSR_SRC_GROUPS ${VSR_SRC_GROUPS} PARENT_SCOPE)

    list(APPEND VSR_HEADER_GROUPS ${DIR_HEADERS})
    set(VSR_HEADER_GROUPS ${VSR_HEADER_GROUPS} PARENT_SCOPE)
endfunction(add_directory)

# src dependencies
aux_source_directory(src VSR_SRC)
source_group("Source Files" FILES ${VSR_SRC})
file(GLOB VSR_HEADERS include/*.h)
source_group("Header Files" FILES ${VSR_HEADERS})

add_directory("RegionGrow")
add_directory("QEM")
add_directory("knn")
add_directory("core")
add_directory("reconstruction")

add_executable(${VSR_name} 
    ${VSR_SRC} ${VSR_HEADERS}
    ${VSR_SRC_GROUPS} ${VSR_HEADER_GROUPS}
)

# compile features
target_compile_features(${VSR_name} 
    PRIVATE cxx_std_20
)

# include directories
target_include_directories(${VSR_name}  
    PRIVATE include
)

# link vendor libraries
target_link_libraries(${VSR_name} 
    PRIVATE Eigen3::Eigen
    PRIVATE CGAL::CGAL
    PRIVATE SCIP_interface
    PRIVATE CGAL::CGAL_Qt5
    PRIVATE CGAL::CGAL_Basic_viewer
)

target_compile_definitions(${VSR_name} 
    PRIVATE CGAL_USE_SCIP
)

# pch
target_precompile_headers(${VSR_name} 
    PRIVATE include/pch.h
)

# set properties
set_target_properties(${VSR_name} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)