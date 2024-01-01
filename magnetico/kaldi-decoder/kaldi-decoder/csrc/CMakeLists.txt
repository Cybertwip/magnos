# Please keep the source files alphabetically sorted
set(srcs
  decodable-ctc.cc
  eigen.cc
  faster-decoder.cc
)


add_library(kaldi-decoder-core ${srcs})

target_link_libraries(kaldi-decoder-core PUBLIC kaldifst_core)

target_include_directories(kaldi-decoder-core PUBLIC ../..)
target_include_directories(kaldi-decoder-core PUBLIC ${AIRSIM_ROOT}/AirLib/thirdparty/eigen3)
