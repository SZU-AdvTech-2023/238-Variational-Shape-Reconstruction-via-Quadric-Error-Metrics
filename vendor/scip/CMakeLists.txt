find_package(SCIP REQUIRED)

# target libscip doesn't have include dirs
# need to add them manually
add_library(SCIP_interface INTERFACE)
target_link_libraries(SCIP_interface INTERFACE libscip)