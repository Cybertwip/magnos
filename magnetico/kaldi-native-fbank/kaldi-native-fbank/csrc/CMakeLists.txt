
set(sources
  feature-fbank.cc
  feature-functions.cc
  feature-window.cc
  fftsg.cc
  mel-computations.cc
  online-feature.cc
  rfft.cc
  whisper-feature.cc
)

if(KALDI_NATIVE_FBANK_ENABLE_CHECK)
  list(APPEND sources log.cc)
endif()

add_library(kaldi-native-fbank-core ${sources})
if(KALDI_NATIVE_FBANK_ENABLE_CHECK)
  target_compile_definitions(kaldi-native-fbank-core PUBLIC KNF_ENABLE_CHECK=1)

  if(KNF_HAVE_EXECINFO_H)
    target_compile_definitions(kaldi-native-fbank-core PRIVATE KNF_HAVE_EXECINFO_H=1)
  endif()

  if(KNF_HAVE_CXXABI_H)
    target_compile_definitions(kaldi-native-fbank-core PRIVATE KNF_HAVE_CXXABI_H=1)
  endif()
endif()

# We are using std::call_once() in log.h,which requires us to link with -pthread
if(NOT WIN32 AND KALDI_NATIVE_FBANK_ENABLE_CHECK)
  target_link_libraries(kaldi-native-fbank-core -pthread)
endif()

target_include_directories(kaldi-native-fbank-core PUBLIC ../../)
