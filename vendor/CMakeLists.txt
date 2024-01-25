add_subdirectory(Eigen)
add_subdirectory(cgal)
add_subdirectory(scip)

if(CGAL_Qt5_FOUND)
    message(STATUS "CGAL Qt5 found in vendor")
    set(CGAL_Qt5_FOUND TRUE PARENT_SCOPE)
endif()