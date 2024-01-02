#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#include <Eigen/Dense>

#include "mlpack.hpp"

#include "phonemize.hpp"
#include "phoneme_ids.hpp"
#include "espeak-ng/speak_lib.h"

#include "sherpa-onnx/csrc/offline-recognizer.h"
#include "sherpa-onnx/csrc/parse-options.h"
#include "sherpa-onnx/csrc/wave-reader.h"


#include "librosa.h"

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <cstddef>


constexpr int WINDOW_SIZE = 2048;  // Adjust as needed
constexpr double SAMPLING_RATE = 44100.0;  // Adjust as needed
constexpr double OVERLAP_RATIO = 0.5;  // Adjust overlap ratio

struct AudioSet {
	std::vector<int64_t> phonemes;
	std::vector<int16_t> audio;
};

struct WordSet {
	std::string word;
	std::tuple<int64_t, int64_t> time;
	std::vector<float> pcm;
};


std::vector<AudioSet> data;
std::vector<WordSet> wordData;


using namespace Eigen;

class AudioProcessor {
private:
	sherpa_onnx::OfflineRecognizerConfig config;
	std::unique_ptr<sherpa_onnx::OfflineRecognizer> recognizer;
	
	std::vector<std::unique_ptr<sherpa_onnx::OfflineStream>> ss;
	std::vector<sherpa_onnx::OfflineStream *> ss_pointers;
	
	int sampling_rate;

public:
	AudioProcessor(const char* filename) : filename(filename) {
		
		std::filesystem::path dataPath = "/opt/homebrew/share";
		std::string espeakDataPath =
		std::filesystem::absolute(
								 dataPath/ "espeak-ng-data").string();

		espeak_Initialize(AUDIO_OUTPUT_SYNCHRONOUS,
									   /*buflength*/ 0,
									   /*path*/ espeakDataPath.c_str(),
									   /*options*/ 0);
		
		// Use espeak-ng for phonemization
		eSpeakConfig.voice = "es";
		
		config.model_config.whisper.task = "transcribe";
		
		config.model_config.whisper.encoder = "./tiny-encoder.onnx";

		config.model_config.whisper.decoder = "./tiny-decoder.onnx";

		config.model_config.tokens = "tiny-tokens.txt";
		
		config.decoding_method = "greedy_search";
		
		config.model_config.whisper.language = "es";
		
		config.model_config.whisper.tail_paddings = 300;

		
		recognizer = std::make_unique<sherpa_onnx::OfflineRecognizer>(config);
		
		initializeOpenAL();
		loadAudioFile();
		playAudio();
		
	}
	
	~AudioProcessor() {
		cleanup();
	}
	
	// Function to normalize a vector of integers to the range [0.0, 1.0]
	std::vector<float> normalize(const std::vector<int16_t>& input) {
		std::vector<float> output(input.size());
		
		// Convert the buffer to floats. (before resampling)
		// Normalize the values to the range [0.0, 1.0]
		std::transform(input.begin(), input.end(), output.begin(),
					   [](int16_t value) {
			
			return static_cast<float>(value) / 32767.0f;
		});
		
		return output;
	}
	
	// Function to denormalize a vector of floats from the range [0.0, 1.0] to integers
	std::vector<int16_t> denormalize(const std::vector<float>& input) {
		std::vector<int16_t> output(input.size());
		
		
		// Denormalize the values to the original range of integers
		std::transform(input.begin(), input.end(), output.begin(),
					   [](float value) {
			int16_t tmp = static_cast<int16_t>(32767 * value);
			
			return static_cast<int16_t>(tmp);
		});
		
		return output;
	}

	void trainModel(std::vector<float> pcmf32) {
		std::string text = "";
		
		std::vector<std::tuple<std::string, int64_t, int64_t>> recognizedSegments;
		
		auto s = recognizer->CreateStream();
		s->AcceptWaveform(sampling_rate, pcmf32.data(), pcmf32.size());
		
		ss.push_back(std::move(s));
		ss_pointers.push_back(ss.back().get());

		recognizer->DecodeStreams(ss_pointers.data(), ss_pointers.size());
		
		auto result = ss_pointers.back()->GetResult();
		
		

//
//		std::string text = "Para entrenar un modelo. Con rapidez, no necesitas una GPU super buena, o una MacBook. De hecho, estoy trabajando en algo sencillo. El español tiene fonemas, como ua, o paahpaah. Por ejemplo. Entonces, para generar un sintetizador de voz, le dices a la computadora ah mira, cuando yo diga agua, tu tienes que verificar que la onda de audio (pe ce emes) o salida de audio, es exactamente (o un aproximado) a la que escuchaste cuando te entrené. El primer paso es convertir audio en fonemas, eso se hace con espik ene je. Después se le pasa un entrenamiento de Machine Learning. Me llevó cierto tiempo pensar en la solución, justo ahora estoy trabajando en ello. No desesperen. Saludos.";
//		
		std::vector<std::vector<piper::Phoneme>> phonemes;
		
		piper::phonemize_eSpeak(text, eSpeakConfig, phonemes);
		
		std::map<piper::Phoneme, std::size_t> missingPhonemes;
		
		std::vector<piper::PhonemeId> phonemeIds;

		for(auto& phoneme : phonemes){
			std::vector<piper::PhonemeId> singlePhonemeIds;

			piper::phonemes_to_ids(phoneme, idConfig, singlePhonemeIds,
			    missingPhonemes);
			
			phonemeIds.insert(phonemeIds.end(), singlePhonemeIds.begin(), singlePhonemeIds.end());
		}
		
//		data.push_back({phonemeIds, pcms});
		
		recognizedSegments.push_back({result.text, result.timestamps[0], result.timestamps[1]});
		
		for(auto& wordSet : recognizedSegments){
			auto [text, t0, t1] = wordSet;
						
			int samplingRate = sampling_rate;

			int64_t sampleT0 = static_cast<int64_t>(t0 * samplingRate);
			int64_t sampleT1 = static_cast<int64_t>(t1 * samplingRate);

			// Ensure sampleT1 is within the range of your pcmf32 vector.
			sampleT1 = std::min(sampleT1, static_cast<int64_t>(pcmf32.size()));
			
			std::vector<float> pcmChunk(pcmf32.begin() + sampleT0, pcmf32.begin() + sampleT1);

			assert(!pcmChunk.empty());
			
			wordData.push_back({text, std::make_tuple(sampleT0, sampleT1), pcmChunk});

		}
		
	}
	void processAudio() {
		while (sourceState == AL_PLAYING) {
			alGetSourcei(source, AL_SOURCE_STATE, &sourceState);
		}
	}

	
private:
	size_t maxPcmSize = 0;
	
	const char* filename;
	ALCdevice* device;
	ALCcontext* context;
	std::vector<int16_t> pcmData;
	std::vector<float> normalPcmData;
	ALuint buffer;
	ALuint source;
	ALint sourceState;
	ALint lastProcessedSampleOffset;
	std::unique_ptr<mlpack::FFN<mlpack::MeanSquaredError>> mlModel;

	void initializeOpenAL() {
		device = alcOpenDevice(nullptr);
		context = alcCreateContext(device, nullptr);
		alcMakeContextCurrent(context);
	}
	
	void loadAudioFile() {
		bool is_ok;

		normalPcmData =
		sherpa_onnx::ReadWave(filename, &sampling_rate, &is_ok);
		
		trainModel(normalPcmData);
		
		// Assuming all audio vectors have the same size
			
		int counter = 0;
		
		maxPcmSize = 0;
		
		// First, find the size of the largest pcm vector
		for (auto& wordData : wordData) {
			std::vector<float> pcm = wordData.pcm;
			if (pcm.size() > maxPcmSize) {
				maxPcmSize = pcm.size();
			}
		}

		arma::mat dataset(maxPcmSize + 2, wordData.size());


		// Map of words to numerical labels
		std::unordered_map<std::string, int> wordToLabel;
		
		for (auto& word : wordData) {
			std::string text = word.word;
			std::vector<float> pcm = word.pcm;
			
					
			std::vector<float> specs;

			int sr = sampling_rate;
			int n_fft = 20;
			int n_hop = 5;
			std::string window = "hann";
			bool center = false;
			std::string pad_mode = "reflect";
			float power = 2.f;
			int n_mel = 40;
			int fmin = 80;
			int fmax = 7600;

			std::vector<std::vector<float>> mels = librosa::Feature::melspectrogram(pcm, sr, n_fft, n_hop, window, center, pad_mode, power,n_mel, fmin, fmax);

			for(auto& spec : mels){
				specs.insert(specs.end(), spec.begin(), spec.end());
			}

			auto [t0, t1] = word.time;
			
			// Check if the word already has a label
			if (wordToLabel.find(text) == wordToLabel.end()) {
				// Assign a new label for the word
				int label = wordToLabel.size();  // This is a simple way to assign unique labels
				wordToLabel[text] = label;
			}
			
			// Get the numerical label for the word
			int label = wordToLabel[text];
			
			// Assuming dataset is of type arma::mat
			dataset(0, counter) = label;
			dataset(1, counter) = std::abs(t1 - t0);
			// Assuming pcm is a vector of numerical values
			// Assuming pcm is a vector of numerical values
			// Assuming pcm is a vector of numerical values
			for (size_t i = 0; i < maxPcmSize; ++i) {
				if (i < pcm.size()) {
					dataset(2 + i, counter) = specs[i];
				} else {
					dataset(2 + i, counter) = 0;  // Or whatever default value makes sense
				}
			}
			
			counter++;
		}

		arma::mat labels = dataset.rows(2, dataset.n_rows - 1);
		dataset.shed_rows(2, dataset.n_rows - 1);

		mlModel = std::make_unique<mlpack::FFN<mlpack::MeanSquaredError>>();
		
		mlModel->Add<mlpack::Linear>(2);  // Input layer to hidden layer.
		mlModel->Add<mlpack::ReLU>();
		// Activation function.
		mlModel->Add<mlpack::Linear>(maxPcmSize);  // Hidden layer to output.

		// Set the model to use the stochastic gradient descent optimizer.
		ens::SGD optimizer(0.01, 32, 10000);

		// Train the model.
		mlModel->Train(dataset, labels, optimizer);

		std::vector<std::vector<piper::Phoneme>> phonemes;
		
		piper::phonemize_eSpeak("Hola mundo", eSpeakConfig, phonemes);

		std::vector<piper::PhonemeId> phonemeIds;
		
		std::map<piper::Phoneme, std::size_t> missingPhonemes;

		for(auto& phoneme : phonemes){
			std::vector<piper::PhonemeId> singlePhonemeIds;
			
			piper::phonemes_to_ids(phoneme, idConfig, singlePhonemeIds,
								   missingPhonemes);
			
			phonemeIds.insert(phonemeIds.end(), singlePhonemeIds.begin(), singlePhonemeIds.end());
		}

		int64_t phonemeValue = arma::accu(arma::conv_to<arma::Row<int64_t>>::from(phonemeIds));

		arma::mat input(2, 1);
		input(0, 0) = 0;
		input(1, 0) = 16000;

		arma::mat output(1, maxPcmSize);
		mlModel->Predict(input, output);
		
		
		// Display the decoded predictions.
		std::cout << "Decoded Predictions: " << output << std::endl;
		
		std::vector<float> pcmOutput(output.begin(), output.end());

		std::vector<int16_t> pcmFinal = denormalize(pcmOutput);

		alGenBuffers(1, &buffer);
		alBufferData(buffer, AL_FORMAT_MONO16, pcmFinal.data(), pcmFinal.size() * sizeof(int16_t), sampling_rate * 1);
		
		alGenSources(1, &source);
		alSourcei(source, AL_BUFFER, buffer);
		
		alSourcef(source, AL_GAIN, 20.0f);

	}
	
	void playAudio() {
		alSourcePlay(source);
		
		while (sourceState != AL_PLAYING) {
			alGetSourcei(source, AL_SOURCE_STATE, &sourceState);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	
	void cleanup() {
		alSourceStop(source);
		alDeleteSources(1, &source);
		alDeleteBuffers(1, &buffer);
		
		alcMakeContextCurrent(nullptr);
		alcDestroyContext(context);
		alcCloseDevice(device);
	}
	
	void calculateSTFT(Eigen::VectorXd& segment) {
		int segmentSize = segment.size();
		
		if (segmentSize == 0) {
			std::cerr << "Error: Segment size is zero." << std::endl;
			return;
		}
		
//		Eigen::VectorXcd fftResult = FFT(segment);
//		
//		if (fftResult.hasNaN()) {
//			std::cerr << "Error: FFT result contains NaN values." << std::endl;
//			return;
//		}
//		
//		fftResult /= segmentSize;
//		
//		double numCycles = (fftResult.size() - 1) * 2.0 * M_PI / segmentSize;
//		double power = fftResult.real().array().square().sum();
//		
//		std::cout << "Number of Wave Cycles: " << numCycles << ", Power: " << power << std::endl;
	}
	
	piper::eSpeakPhonemeConfig eSpeakConfig;
	
	piper::PhonemeIdConfig idConfig;


};
int main(int argc, char** argv) {
	const char* filename = "./training.wav";
	AudioProcessor audioProcessor(filename);
	audioProcessor.processAudio();

	return 0;
}
