cmake_minimum_required(VERSION 3.10.2)

project (m12_gnss)

find_package(RTIConnextDDS REQUIRED
HINTS ${CMAKE_SOURCE_DIR})


set(DDS_TYPE_OBJECT_DIR
${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/objs
)

#
file(GLOB dds_typeobjs
${DDS_TYPE_OBJECT_DIR}/*.o
)
message(
"The DDS_TYPE_OBJECT_DIR is: ${DDS_TYPE_OBJECT_DIR}"
)
message(
"The object files are: ${dds_typeobjs}"
)



add_executable(publish_gnss
${CMAKE_SOURCE_DIR}/src/publish_gnss.cpp
${dds_typeobjs}
)
target_compile_definitions(publish_gnss PUBLIC
-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT
)


target_include_directories( publish_gnss PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/IDL/M12Typelib/include
  ${CONNEXTDDS_INCLUDE_DIRS}
)


target_link_libraries( publish_gnss PUBLIC
${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
pthread
dl
)

#[[
message(
  "CONNEXTDDS_C_API_LIBRARIES: ${CONNEXTDDS_C_API_LIBRARIES_RELEASE_STATIC}"
)
]]
#[[
target_link_libraries( publish_gnss PRIVATE
  ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
  pthread
  dl
)
]]
