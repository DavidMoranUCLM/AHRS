idf_component_register(SRCS ""
                      INCLUDE_DIRS "include")

#add_dependencies(${COMPONENT_LIB} KALMAN)



set(LIBRARY_DIR ${CMAKE_CURRENT_LIST_DIR}/lib)

# Find all `.a` files in the directory
file(GLOB LIBRARIES "${LIBRARY_DIR}/*.a")

target_link_libraries(${COMPONENT_TARGET} INTERFACE ${LIBRARIES})

