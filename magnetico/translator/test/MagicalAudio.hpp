#pragma once

#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#include "sherpa-onnx/csrc/offline-recognizer.h"
#include "sherpa-onnx/csrc/parse-options.h"
#include "sherpa-onnx/csrc/wave-reader.h"
#include "sherpa-onnx/csrc/offline-tts.h"
#include "sherpa-onnx/csrc/wave-writer.h"
#include "sherpa-onnx/csrc/resample.h"
#include "FileLoop.h"
#include "MagicalWhisper.hpp"
#include <filesystem>
#include <cassert>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

class MagicalAudio {
public:
    struct WordSet {
        std::string word;
        std::tuple<int64_t, int64_t> time;
        std::vector<float> pcm;
    };

private:
    sherpa_onnx::OfflineRecognizerConfig config;
    std::unique_ptr<sherpa_onnx::OfflineRecognizer> recognizer;

    std::vector<std::unique_ptr<sherpa_onnx::OfflineStream>> ss;
    std::vector<sherpa_onnx::OfflineStream *> ss_pointers;

public:
	MagicalAudio(const char *filename);

    ~MagicalAudio();

    std::vector<float> normalize(const std::vector<int16_t> &input);

    std::vector<int16_t> denormalize(const std::vector<float> &input);

    std::vector<WordSet> splitPcm(std::vector<float> pcmf32, int sampling_rate);

    void processAudio();

private:
    size_t maxPcmSize = 0;

    const char *filename;
    ALCdevice *device;
    ALCcontext *context;
    std::vector<int16_t> pcmData;
    ALuint buffer;
    ALuint source;
    ALint sourceState;

    void initializeOpenAL();

	std::vector<float> loadAudioFile(const std::string& filename, int& sampling_rate);
	
	std::vector<int16_t> speechify(std::vector<float> inputPcm, std::vector<MagicalAudio::WordSet> wordData, int sampling_rate);
	
    void playAudio(std::vector<int16_t>& pcm, int sampling_rate, int channels);

    void cleanup();
	
	std::unique_ptr<MagicalWhisper> magicalWhisper;

};
