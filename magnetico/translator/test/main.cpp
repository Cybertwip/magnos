#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#include <sndfile.h>
#include <Eigen/Dense>

#include "FFT.hpp"
#include "mlpack.hpp"
#include "phonemize.hpp"
#include "phoneme_ids.hpp"
#include "espeak-ng/speak_lib.h"
#include "whisper.h"

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
	struct whisper_context_params cparams;
	struct whisper_context * ctx;
	whisper_full_params wparams;

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

		wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
		
		wparams.language = "es";
		
		wparams.n_threads = 10;
		
		wparams.split_on_word = true;
		
		wparams.max_len = 10;
		
		wparams.token_timestamps = true;
//		wparams.speed_up = true;
		
		cparams.use_gpu = true;
		ctx = whisper_init_from_file_with_params("ggml-tiny.bin", cparams);
		
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
		const float div = (1.0f/32768.0f);
		
		// Normalize the values to the range [0.0, 1.0]
		std::transform(input.begin(), input.end(), output.begin(),
					   [&div](int16_t value) {
			
			return static_cast<float>(value) * div;
		});
		
		return output;
	}
	
	// Function to denormalize a vector of floats from the range [0.0, 1.0] to integers
	std::vector<int16_t> denormalize(const std::vector<float>& input) {
		std::vector<int16_t> output(input.size());
		
		const float mul = (32768.0f);
		
		// Denormalize the values to the original range of integers
		std::transform(input.begin(), input.end(), output.begin(),
					   [&mul](float value) {
			
			int32_t tmp = static_cast<int32_t>(mul * value);
			
			tmp = std::max( tmp, -32768 ); // CLIP < 32768
			tmp = std::min( tmp, 32767 );  // CLIP > 32767

			return tmp;
		});
		
		return output;
	}


	void trainModel(std::vector<int16_t> pcms) {
		
		std::vector<float> pcmf32 = normalize(pcms);
		
		whisper_full_parallel(ctx, wparams, pcmf32.data(), pcmf32.size(), 8);
		
		const int n_segments = whisper_full_n_segments(ctx);
		
		std::string text = "";
		
		std::vector<std::tuple<std::string, int64_t, int64_t>> recognizedSegments;

		
		for (int i = 0; i < n_segments; ++i) {
			const std::string chain = whisper_full_get_segment_text(ctx, i);
			
			const int64_t t0 = whisper_full_get_segment_t0(ctx, i);
			const int64_t t1 = whisper_full_get_segment_t1(ctx, i);

			text += chain;
			
			recognizedSegments.emplace_back(std::make_tuple(chain, t0, t1));

		}
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
		
		data.push_back({phonemeIds, pcms});
		
		for(auto& wordSet : recognizedSegments){
			auto [text, t0, t1] = wordSet;
			
			int64_t finalt0 = t0 > t1 ? t1 : t0;
			
			int64_t finalt1 = t0 > t1 ? t0 : t1;
			
			int samplingRate = 16000;

			int64_t sampleT0 = static_cast<int64_t>(finalt0 * 0.001 * samplingRate);
			int64_t sampleT1 = static_cast<int64_t>(finalt1 * 0.001 * samplingRate);

			// Ensure sampleT1 is within the range of your pcmf32 vector.
			sampleT1 = std::min(sampleT1, static_cast<int64_t>(pcmf32.size()));
			
			std::vector<float> pcmChunk(pcmf32.begin() + sampleT0, pcmf32.begin() + sampleT1);

			wordData.push_back({text, std::make_tuple(finalt0, finalt1), pcmChunk});

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
	SF_INFO sfInfo;
	SNDFILE* sndfile;
	std::vector<int16_t> pcmData;
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
		sndfile = sf_open(filename, SFM_READ, &sfInfo);
		
		if (!sndfile || sfInfo.frames <= 0) {
			std::cerr << "Error opening/reading audio file: " << filename << std::endl;
			cleanup();
			std::exit(1);
		}
		
		pcmData.resize(sfInfo.frames * sfInfo.channels);
		sf_read_short(sndfile, &pcmData[0], sfInfo.frames * sfInfo.channels);
		sf_close(sndfile);
		
		trainModel(pcmData);
		
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
		
		for (auto& wordData : wordData) {
			std::string word = wordData.word;
			std::vector<float> pcm = wordData.pcm;
			
			auto [t0, t1] = wordData.time;
			
			// Check if the word already has a label
			if (wordToLabel.find(word) == wordToLabel.end()) {
				// Assign a new label for the word
				int label = wordToLabel.size();  // This is a simple way to assign unique labels
				wordToLabel[word] = label;
			}
			
			// Get the numerical label for the word
			int label = wordToLabel[word];
			
			// Assuming dataset is of type arma::mat
			dataset(0, counter) = label;
			dataset(1, counter) = std::abs(t1 - t0);
			// Assuming pcm is a vector of numerical values
			// Assuming pcm is a vector of numerical values
			// Assuming pcm is a vector of numerical values
			for (size_t i = 0; i < maxPcmSize; ++i) {
				if (i < pcm.size()) {
					dataset(2 + i, counter) = pcm[i];
				} else {
					dataset(2 + i, counter) = 0;  // Or whatever default value makes sense
				}
			}
			
			counter++;
		}

		arma::mat labels = dataset.rows(2, dataset.n_rows - 1);
		dataset.shed_rows(2, dataset.n_rows - 1);

		mlModel = std::make_unique<mlpack::FFN<mlpack::MeanSquaredError>>();
		
		mlModel->Add<mlpack::Linear>(32);  // Input layer to hidden layer.
		mlModel->Add<mlpack::ReLU>();
		// Activation function.
		mlModel->Add<mlpack::Linear>(maxPcmSize);  // Hidden layer to output.

		// Set the model to use the stochastic gradient descent optimizer.
		ens::SGD optimizer(0.01, 1, 1000 /* Number of iterations */);

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
		input(0, 0) = 4;
		input(1, 0) = 30;

		arma::mat output(1, maxPcmSize);
		mlModel->Predict(input, output);

		arma::vec outputVector = arma::vectorise(output);
		
		std::vector<float> pcmOutput = arma::conv_to<std::vector<float>>::from(arma::vectorise(output));
		
		std::cout << "Output Matrix:\n" << output << std::endl;

		std::vector<int16_t> pcmFinal = denormalize(pcmOutput);

		alGenBuffers(1, &buffer);
		alBufferData(buffer, AL_FORMAT_MONO16, pcmFinal.data(), pcmFinal.size() * sizeof(int16_t), sfInfo.samplerate * sfInfo.channels);
		
		alGenSources(1, &source);
		alSourcei(source, AL_BUFFER, buffer);
		
		alSourcef(source, AL_GAIN, 4.0f);

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
		
		Eigen::VectorXcd fftResult = FFT(segment);
		
		if (fftResult.hasNaN()) {
			std::cerr << "Error: FFT result contains NaN values." << std::endl;
			return;
		}
		
		fftResult /= segmentSize;
		
		double numCycles = (fftResult.size() - 1) * 2.0 * M_PI / segmentSize;
		double power = fftResult.real().array().square().sum();
		
		std::cout << "Number of Wave Cycles: " << numCycles << ", Power: " << power << std::endl;
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
