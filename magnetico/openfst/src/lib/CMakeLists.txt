FILE(GLOB HEADER_FILES ../include/fst/*.h)

add_library(fst
  compat.cc
  flags.cc
  fst-types.cc
  fst.cc
  mapped-file.cc
  properties.cc
  symbol-table.cc
  symbol-table-ops.cc
  util.cc
  weight.cc
  ${HEADER_FILES}
)
set_target_properties(fst PROPERTIES
  SOVERSION "${SOVERSION}"
)

if(WIN32)
  target_link_options(fst INTERFACE /WHOLEARCHIVE:$<TARGET_LINKER_FILE:fst>)
endif()

target_include_directories(fst PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include)