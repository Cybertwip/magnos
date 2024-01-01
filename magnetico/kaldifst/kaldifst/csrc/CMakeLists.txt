
set(srcs
  context-fst.cc
  kaldi-fst-io.cc
  kaldi-holder.cc
  kaldi-io.cc
  kaldi-math.cc
  kaldi-semaphore.cc
  kaldi-table.cc
  parse-options.cc
  text-normalizer.cc
  text-utils.cc
  )

add_library(kaldifst_core ${srcs})
target_link_libraries(kaldifst_core PUBLIC fst)

target_include_directories(kaldifst_core PUBLIC ../../)
