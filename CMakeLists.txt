cmake_minimum_required(VERSION 3.0)

project(rmp440-genom-infuse)

set(NAME rmp440-infuse)
set(GENOM_FILE rmp440.gen)

set(${NAME}_FILES)
file(GLOB_RECURSE ${NAME}_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cc" "*.hpp" "*.cpp" "*.yaml"  "*.ac" "*.am" "*.h" "*.c")
list(APPEND ${NAME}_FILES "${GENOM_FILE}")

file(GLOB_RECURSE IDL_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.idl")
list(APPEND ${NAME}_FILES "${IDL_FILES}")


set(${NAME}_FILES_IN_BINARY_DIR)
foreach(file ${${NAME}_FILES})
	configure_file(${file} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${file}  COPYONLY)
	list(APPEND ${NAME}_FILES_IN_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${file})
endforeach()

# Install genom module inside build dir
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}_genom3.stamp
	COMMAND PKG_CONFIG_PATH=${CMAKE_BINARY_DIR}/pkgconfig:${CMAKE_BINARY_DIR}/genom_modules/lib/pkgconfig:$ENV{PKG_CONFIG_PATH} genom3 skeleton -l c++ -i "${GENOM_FILE}"
	#COMMAND PKG_CONFIG_PATH=${CMAKE_BINARY_DIR}/pkgconfig:${CMAKE_BINARY_DIR}/genom_modules/lib/pkgconfig:$ENV{PKG_CONFIG_PATH} genom3 skeleton -l c -i "${GENOM_FILE}"
	COMMAND ./bootstrap.sh
	COMMAND PKG_CONFIG_PATH=${CMAKE_BINARY_DIR}/pkgconfig:${CMAKE_BINARY_DIR}/genom_modules/lib/pkgconfig:$ENV{PKG_CONFIG_PATH} ./configure --prefix="${GENOM_INSTALL_PREFIX}" --with-templates=ros/server,ros/client/ros,ros/client/c CXXFLAGS='-std=c++11'
	COMMAND make ${GENOM_NUM_JOBS} install
	#COMMAND make install
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/${NAME}_genom3.stamp
	DEPENDS ${${NAME}_FILES}
	COMMENT "Installing GenoM3 module ${NAME} on ${GENOM_INSTALL_PREFIX}"
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
)
add_custom_target(${NAME}_genom3 ALL DEPENDS
	${CMAKE_CURRENT_BINARY_DIR}/${NAME}_genom3.stamp
	infuse_types_genom3
	${CMAKE_BINARY_DIR}/Common/Types/InfuseTypes/infuse_types_genom3.stamp)

# # Add rules to install the genom module in the system
# install(CODE "execute_process(
# 	COMMAND ${CMAKE_BINARY_DIR}/install_genom_module.sh
# 	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME})")
