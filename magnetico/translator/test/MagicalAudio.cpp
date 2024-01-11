#include "MagicalAudio.hpp"
#include "PitShift.h"

#include <vector>
#include <cmath>


namespace viper {

class PitchYIN {
private:
	std::vector<float> buffer;
	int bufferSize;
	float threshold;
	
public:
	PitchYIN(size_t size, int rate, float regression) : bufferSize(size), threshold(regression) {
		buffer.resize(size);
	}
	
	float calculatePitch(std::vector<float>& pcm) {
		// Step 1: Calculate the difference function
		for (int tau = 0; tau < bufferSize; tau++) {
			buffer[tau] = 0;
			for (int i = 0; i < bufferSize - tau; i++) {
				float delta = pcm[i] - pcm[i + tau];
				buffer[tau] += delta * delta;
			}
		}
		
		// Step 2: Cumulative mean normalized difference function
		buffer[0] = 1;
		for (int tau = 1; tau < bufferSize; tau++) {
			buffer[tau] *= tau;
			buffer[tau] += buffer[tau - 1];
			if (buffer[tau] > 0) {
				buffer[tau] = buffer[tau - 1] / buffer[tau];
			} else {
				buffer[tau] = 1;
			}
		}
		
		// Step 3: Absolute threshold
		int tau;
		for (tau = 1; tau < bufferSize && buffer[tau] > threshold; tau++) {}
		
		// Step 4: Interpolate around the minimum
		float betterTau;
		if (tau < bufferSize) {
			int x0 = (tau < 1) ? tau : tau - 1;
			int x2 = (tau + 1 < bufferSize) ? tau + 1 : tau;
			if (x0 == tau) {
				if (buffer[tau] < buffer[x2]) {
					betterTau = tau;
				} else {
					betterTau = x2;
				}
			} else if (x2 == tau) {
				if (buffer[tau] < buffer[x0]) {
					betterTau = tau;
				} else {
					betterTau = x0;
				}
			} else {
				float s0 = buffer[x0];
				float s1 = buffer[tau];
				float s2 = buffer[x2];
				betterTau = tau + (s2 - s0) / (2 * (2 * s1 - s2 - s0));
			}
		} else {
			betterTau = tau;
		}
		
		// Convert to frequency
		return 1.0f / betterTau;
	}
};


}

MagicalAudio::MagicalAudio(const char *filename) : filename(filename) {
    std::filesystem::path dataPath = "/opt/homebrew/share";
    std::string espeakDataPath =
        std::filesystem::absolute(
            dataPath / "espeak-ng-data").string();

    config.model_config.whisper.task = "transcribe";
    config.model_config.whisper.encoder = "./tiny-encoder.onnx";
    config.model_config.whisper.decoder = "./tiny-decoder.onnx";
    config.model_config.tokens = "tiny-tokens.txt";
    config.decoding_method = "greedy_search";
    config.model_config.whisper.language = "es";
    config.model_config.whisper.tail_paddings = 300;

    recognizer = std::make_unique<sherpa_onnx::OfflineRecognizer>(config);

    initializeOpenAL();
	
	int samplingRate = 0;
    auto normalAudioPcm = loadAudioFile(filename, samplingRate);
	
	auto wordData = splitPcm(normalAudioPcm, samplingRate);
	
//	auto textToSpeechSamples = speechify(normalAudioPcm, wordData, samplingRate);

	auto audio = denormalize(wordData[7].pcm);
    playAudio(audio, samplingRate, 1);
}

MagicalAudio::~MagicalAudio() {
    cleanup();
}

std::vector<float> MagicalAudio::normalize(const std::vector<int16_t> &input) {
    std::vector<float> output(input.size());

    std::transform(input.begin(), input.end(), output.begin(),
                   [](int16_t value) {
                       return static_cast<float>(value) / 32767.0f;
                   });

    return output;
}

std::vector<int16_t> MagicalAudio::denormalize(const std::vector<float> &input) {
    std::vector<int16_t> output(input.size());

    std::transform(input.begin(), input.end(), output.begin(),
                   [](float value) {
                       int16_t tmp = static_cast<int16_t>(32767 * value);
                       return static_cast<int16_t>(tmp);
                   });

    return output;
}

std::vector<MagicalAudio::WordSet> MagicalAudio::splitPcm(std::vector<float> pcmf32, int sampling_rate) {
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

        int64_t sampleT0 = static_cast<int64_t>(t0);
        int64_t sampleT1 = static_cast<int64_t>(t1);
        sampleT1 = std::min(sampleT1, static_cast<int64_t>(pcmf32.size()));

        std::vector<float> pcmChunk(pcmf32.begin() + static_cast<int64_t>(sampleT0 * 0.001 * sampling_rate), pcmf32.begin() + static_cast<int64_t>(sampleT1 * 0.001 * sampling_rate));

        assert(!pcmChunk.empty());

        wordData.push_back({text, std::make_tuple(sampleT0, sampleT1), pcmChunk});
    }

    return wordData;
}

void MagicalAudio::processAudio() {
    while (sourceState == AL_PLAYING) {
        alGetSourcei(source, AL_SOURCE_STATE, &sourceState);
    }
}

void MagicalAudio::initializeOpenAL() {
    device = alcOpenDevice(nullptr);
    context = alcCreateContext(device, nullptr);
    alcMakeContextCurrent(context);
}

std::vector<float> MagicalAudio::loadAudioFile(const std::string& filename, int& sampling_rate) {
	std::vector<float> normalPcmData;
	
    bool is_ok;

    normalPcmData =
        sherpa_onnx::ReadWave(filename.c_str(), &sampling_rate, &is_ok);

	
	return normalPcmData;
}

std::vector<int16_t> MagicalAudio::speechify(std::vector<float> inputPcm, std::vector<MagicalAudio::WordSet> wordData, int sampling_rate){
	
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
	
	float min_freq =
	std::min<int32_t>(sampling_rate, audio.sample_rate);
	float lowpass_cutoff = 0.99 * 0.5 * min_freq;
	
	int32_t lowpass_filter_width = 6;
	auto resampler = std::make_unique<sherpa_onnx::LinearResample>(
					   					audio.sample_rate, sampling_rate, lowpass_cutoff, lowpass_filter_width);
	
	std::vector<float> samples;
	resampler->Resample(audio.samples.data(), audio.samples.size(), true, &samples);
	
	// Begin speech processing
	
	// Set the sample rate (16 kHz)
	stk::Stk::setSampleRate(sampling_rate);
	
	// Create a PitchYin object
	stk::PitShift shifter;
	
	
	for(auto& data : wordData){
		viper::PitchYIN pitchYIN(data.pcm.size(), sampling_rate, 1.0f);

		float inputPitch = pitchYIN.calculatePitch(data.pcm);

		shifter.setShift(inputPitch);

	}
	
	
	for(size_t i = 0; i < samples.size(); ++i){
		float sample = samples[i];
		samples[i] = shifter.tick(sample);
	}
	
	// End speech processing
	return denormalize(samples);
}

void MagicalAudio::playAudio(std::vector<int16_t>& pcm, int sampling_rate, int channels) {
	alGenBuffers(1, &buffer);
	alBufferData(buffer, AL_FORMAT_MONO16, pcm.data(), pcm.size() * sizeof(int16_t), sampling_rate * channels);
	
	alGenSources(1, &source);
	alSourcei(source, AL_BUFFER, buffer);
	
	alSourcef(source, AL_GAIN, 20.0f);
    alSourcePlay(source);

    while (sourceState != AL_PLAYING) {
        alGetSourcei(source, AL_SOURCE_STATE, &sourceState);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MagicalAudio::cleanup() {
    alSourceStop(source);
    alDeleteSources(1, &source);
    alDeleteBuffers(1, &buffer);

    alcMakeContextCurrent(nullptr);
    alcDestroyContext(context);
    alcCloseDevice(device);
}
