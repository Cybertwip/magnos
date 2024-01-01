# Please keep the source files alphabetically sorted
set(srcs
  decodable-ctc.cc
  eigen.cc
  faster-decoder.cc
)


add_library(kaldi-decoder-core ${srcs})

target_link_libraries(kaldi-decoder-core PUBLIC kaldifst_core)
target_link_libraries(kaldi-decoder-core PUBLIC Eigen3::Eigen)

target_include_directories(kaldi-decoder-core PUBLIC ../..)
