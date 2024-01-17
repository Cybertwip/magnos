#include "AudioProcessor.hpp"

AudioProcessor::AudioProcessor(const char *filename) : filename(filename) {
    std::filesystem::path dataPath = "/opt/homebrew/share";
    std::string espeakDataPath =
        std::filesystem::absolute(
            dataPath / "espeak-ng-data").string();

    config.model_config.whisper.task = "transcribe";
    config.model_config.whisper.encoder = "./tiny-encoder.onnx";
    config.model_config.whisper.decoder = "./tiny-decoder.onnx";
    config.model_config.tokens = "./tiny-tokens.txt";
    config.decoding_method = "greedy_search";
    config.model_config.whisper.language = "es";
    config.model_config.whisper.tail_paddings = 300;

    recognizer = std::make_unique<sherpa_onnx::OfflineRecognizer>(config);

    initializeOpenAL();
    loadAudioFile();
    playAudio();
}

AudioProcessor::~AudioProcessor() {
    cleanup();
}

std::vector<float> AudioProcessor::normalize(const std::vector<int16_t> &input) {
    std::vector<float> output(input.size());

    std::transform(input.begin(), input.end(), output.begin(),
                   [](int16_t value) {
                       return static_cast<float>(value) / 32767.0f;
                   });

    return output;
}

std::vector<int16_t> AudioProcessor::denormalize(const std::vector<float> &input) {
    std::vector<int16_t> output(input.size());

    std::transform(input.begin(), input.end(), output.begin(),
                   [](float value) {
                       int16_t tmp = static_cast<int16_t>(32767 * value);
                       return static_cast<int16_t>(tmp);
                   });

    return output;
}

std::vector<AudioProcessor::WordSet> AudioProcessor::splitPcm(std::vector<float> pcmf32) {
    std::vector<WordSet> wordData;
    std::string text = "";
    std::vector<std::tuple<std::string, int64_t, int64_t>> recognizedSegments;

    auto s = recognizer->CreateStream();
    s->AcceptWaveform(sampling_rate, pcmf32.data(), pcmf32.size());

    ss.push_back(std::move(s));
    ss_pointers.push_back(ss.back().get());

    recognizer->DecodeStreams(ss_pointers.data(), ss_pointers.size());

    auto result = ss_pointers.back()->GetResult();

    for (uint32_t i = 0; i < result.words.size(); ++i) {
        auto word = result.words[i];
        auto timestamp = result.wordTimeStamps[i];

        recognizedSegments.push_back({word, timestamp.start, timestamp.end});
    }

    for (auto &wordSet : recognizedSegments) {
        auto [text, t0, t1] = wordSet;

        int samplingRate = sampling_rate;
        int64_t sampleT0 = static_cast<int64_t>(t0 * 0.001f * samplingRate);
        int64_t sampleT1 = static_cast<int64_t>(t1 * 0.001f * samplingRate);
        sampleT1 = std::min(sampleT1, static_cast<int64_t>(pcmf32.size()));

        std::vector<float> pcmChunk(pcmf32.begin() + sampleT0, pcmf32.begin() + sampleT1);

        assert(!pcmChunk.empty());

        wordData.push_back({text, std::make_tuple(sampleT0, sampleT1), pcmChunk});
    }

    return wordData;
}

void AudioProcessor::processAudio() {
    while (sourceState == AL_PLAYING) {
        alGetSourcei(source, AL_SOURCE_STATE, &sourceState);
    }
}

void AudioProcessor::initializeOpenAL() {
    device = alcOpenDevice(nullptr);
    context = alcCreateContext(device, nullptr);
    alcMakeContextCurrent(context);
}

void AudioProcessor::loadAudioFile() {
    bool is_ok;

    normalPcmData =
        sherpa_onnx::ReadWave(filename, &sampling_rate, &is_ok);

    auto wordData = splitPcm(normalPcmData);

    sherpa_onnx::OfflineTtsConfig config;
    config.model.vits.model = "./sharvard-medium.onnx";
    config.model.vits.tokens = "./sharvard-tokens.txt";
    config.model.vits.data_dir = "/opt/homebrew/share/espeak-ng-data";
    sherpa_onnx::OfflineTts tts(config);

    std::string decodedText = "";

    for (auto &data : wordData) {
        decodedText += data.word;
    }

    auto audio = tts.Generate(decodedText, 1);

    std::vector<int16_t> pcmFinal = denormalize(audio.samples);

    alGenBuffers(1, &buffer);
    alBufferData(buffer, AL_FORMAT_MONO16, pcmFinal.data(), pcmFinal.size() * sizeof(int16_t), audio.sample_rate * 1);

    alGenSources(1, &source);
    alSourcei(source, AL_BUFFER, buffer);

    alSourcef(source, AL_GAIN, 20.0f);
}

void AudioProcessor::playAudio() {
    alSourcePlay(source);

    while (sourceState != AL_PLAYING) {
        alGetSourcei(source, AL_SOURCE_STATE, &sourceState);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void AudioProcessor::cleanup() {
    alSourceStop(source);
    alDeleteSources(1, &source);
    alDeleteBuffers(1, &buffer);

    alcMakeContextCurrent(nullptr);
    alcDestroyContext(context);
    alcCloseDevice(device);
}
