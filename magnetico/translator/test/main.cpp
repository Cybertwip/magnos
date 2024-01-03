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

#include "sherpa-onnx/csrc/offline-tts.h"
#include "sherpa-onnx/csrc/parse-options.h"
#include "sherpa-onnx/csrc/wave-writer.h"

#include "sherpa-onnx/csrc/resample.h"


#include "librosapp.hpp"

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

std::vector<float> griffinLim(const std::vector<std::vector<float>>& magnitudeSpectrogram, int nIter = 100) {
	const int nFrames = magnitudeSpectrogram.size();
	const int nFFT = magnitudeSpectrogram[0].size();
	const int hopSize = nFFT / 2;
	
	// Initialize a random phase for the first iteration
	std::vector<std::vector<float>> phase(nFrames, std::vector<float>(nFFT));
	for (auto& frame : phase) {
		for (auto& value : frame) {
			value = static_cast<float>(std::rand()) / RAND_MAX * 2.0f * M_PI - M_PI;
		}
	}
	
	// Griffin-Lim iteration
	for (int iter = 0; iter < nIter; ++iter) {
		// Inverse STFT to obtain time-domain signal
		std::vector<float> reconstructedSignal(nFrames * hopSize, 0.0);
		for (int t = 0; t < nFrames; ++t) {
			for (int f = 0; f < nFFT; ++f) {
				reconstructedSignal[t * hopSize + f] += magnitudeSpectrogram[t][f] * std::cos(phase[t][f]);
			}
		}
		
		// Synthesize the spectrogram from the reconstructed signal
		
		librosa::stft_arg arg;
		arg.y = reconstructedSignal;
		arg.n_fft = nFFT;
		arg.hop_length = hopSize;
		
		auto reconstructedSpectrogram = librosa::stft(&arg);
		
		// Update phase using the angle of the new spectrogram
		// Update phase using the angle of the new spectrogram
		for (int t = 0; t < reconstructedSpectrogram.size(); ++t) {
			for (int f = 0; f < reconstructedSpectrogram[t].size(); ++f) {
				phase[t][f] = std::arg(std::complex<float>(
														   reconstructedSpectrogram[t][f].r,
														   reconstructedSpectrogram[t][f].i
														   ));
			}
		}

	}
	
	// Flatten the reconstructed signal
	std::vector<float> flattenedSignal(nFrames * hopSize);
	for (int t = 0; t < nFrames; ++t) {
		for (int f = 0; f < nFFT; ++f) {
			flattenedSignal[t * hopSize + f] = magnitudeSpectrogram[t][f] * std::cos(phase[t][f]);
		}
	}
	
	return flattenedSignal;
}



std::vector<AudioSet> data;
std::vector<WordSet> wordData;

namespace viper {

// Custom layer mimicking the behavior of the DecoderPreNet in Python.
template<typename DataType = arma::mat>
class DecoderPreNetLayer : public mlpack::Layer<DataType>
{
public:
	// Constructor.
	DecoderPreNetLayer(const size_t inputSize, const size_t outputSize)
	: linear1(inputSize),
	relu1(),
	dropout1(0.5),
	linear2(outputSize),
	relu2(),
	dropout2(0.5)
	{
		// Initialize layers.
	}
	
	
	//! Clone the DecoderPreNetLayer object. This handles polymorphism correctly.
	DecoderPreNetLayer* Clone() const { return new DecoderPreNetLayer(*this); }
	
	// Virtual destructor.
	virtual ~DecoderPreNetLayer() { }
	
	//! Copy the given DecoderPreNetLayer.
	DecoderPreNetLayer(const DecoderPreNetLayer& other)
	: linear1(other.linear1), relu1(other.relu1), dropout1(other.dropout1),
	linear2(other.linear2), relu2(other.relu2), dropout2(other.dropout2),
	linear1Output(other.linear1Output), relu1Output(other.relu1Output),
	dropout1Output(other.dropout1Output), linear2Output(other.linear2Output),
	relu2Output(other.relu2Output)
	{
		// Nothing to do.
	}
	
	//! Take ownership of the given DecoderPreNetLayer.
	DecoderPreNetLayer(DecoderPreNetLayer&& other)
	: linear1(std::move(other.linear1)), relu1(std::move(other.relu1)),
	dropout1(std::move(other.dropout1)), linear2(std::move(other.linear2)),
	relu2(std::move(other.relu2)), dropout2(std::move(other.dropout2)),
	linear1Output(std::move(other.linear1Output)),
	relu1Output(std::move(other.relu1Output)),
	dropout1Output(std::move(other.dropout1Output)),
	linear2Output(std::move(other.linear2Output)),
	relu2Output(std::move(other.relu2Output))
	{
		// Nothing to do.
	}
	
	//! Copy the given DecoderPreNetLayer.
	DecoderPreNetLayer& operator=(const DecoderPreNetLayer& other)
	{
		if (&other != this)
		{
			linear1 = other.linear1;
			relu1 = other.relu1;
			dropout1 = other.dropout1;
			linear2 = other.linear2;
			relu2 = other.relu2;
			dropout2 = other.dropout2;
			
			linear1Output = other.linear1Output;
			relu1Output = other.relu1Output;
			dropout1Output = other.dropout1Output;
			linear2Output = other.linear2Output;
			relu2Output = other.relu2Output;
		}
		
		return *this;
	}
	
	//! Take ownership of the given DecoderPreNetLayer.
	DecoderPreNetLayer& operator=(DecoderPreNetLayer&& other)
	{
		if (&other != this)
		{
			linear1 = std::move(other.linear1);
			relu1 = std::move(other.relu1);
			dropout1 = std::move(other.dropout1);
			linear2 = std::move(other.linear2);
			relu2 = std::move(other.relu2);
			dropout2 = std::move(other.dropout2);
			
			linear1Output = std::move(other.linear1Output);
			relu1Output = std::move(other.relu1Output);
			dropout1Output = std::move(other.dropout1Output);
			linear2Output = std::move(other.linear2Output);
			relu2Output = std::move(other.relu2Output);
		}
		
		return *this;
	}

	// Forward pass.
	void Forward(const DataType& input, DataType& output)
	{
		// First linear layer.
		linear1.Forward(input, linear1Output);
		relu1.Forward(linear1Output, relu1Output);
		dropout1.Forward(relu1Output, dropout1Output);
		
		// Second linear layer.
		linear2.Forward(dropout1Output, linear2Output);
		relu2.Forward(linear2Output, relu2Output);
		dropout2.Forward(relu2Output, output);
	}
	
private:
	mlpack::ann::Linear linear1;
	mlpack::ann::ReLU relu1;
	mlpack::ann::Dropout dropout1;
	
	mlpack::ann::Linear linear2;
	mlpack::ann::ReLU relu2;
	mlpack::ann::Dropout dropout2;
	
	// Intermediate outputs between layers.
	DataType linear1Output;
	DataType relu1Output;
	DataType dropout1Output;
	
	DataType linear2Output;
	DataType relu2Output;
};


// Custom layer mimicking the behavior of the DecoderPreNet in Python.
template<typename DataType = arma::mat>
class IdentityDecoderLayerType : public mlpack::Layer<DataType>
{
private:
	arma::mat linear1Weights;
	arma::mat linear2Weights;
public:
	// Constructor.
	IdentityDecoderLayerType(const size_t inputSize, const size_t outputSize)
	: linear1(inputSize, outputSize),
	relu1(),
	linear2(inputSize, outputSize)
	{
		
		linear1.Weight() = arma::randu<arma::mat>(outputSize, outputSize);
		linear1.Bias() = arma::randu<arma::mat>(outputSize, 1);

	}
	
	
	//! Clone the DecoderPreNetLayer object. This handles polymorphism correctly.
	IdentityDecoderLayerType* Clone() const { return new IdentityDecoderLayerType(*this); }
	
	// Virtual destructor.
	virtual ~IdentityDecoderLayerType() { }
	
	//! Copy the given DecoderPreNetLayer.
	IdentityDecoderLayerType(const IdentityDecoderLayerType& other)
	: linear1(other.linear1), relu1(other.relu1),
	linear2(other.linear2),
	linear1Output(other.linear1Output), relu1Output(other.relu1Output)
	{
		// Nothing to do.
	}
	
	//! Take ownership of the given DecoderPreNetLayer.
	IdentityDecoderLayerType(IdentityDecoderLayerType&& other)
	: linear1(std::move(other.linear1)), relu1(std::move(other.relu1)),
	linear1Output(std::move(other.linear1Output)),
	relu1Output(std::move(other.relu1Output))
	{
		// Nothing to do.
	}
	
	//! Copy the given DecoderPreNetLayer.
	IdentityDecoderLayerType& operator=(const IdentityDecoderLayerType& other)
	{
		if (&other != this)
		{
			linear1 = other.linear1;
			relu1 = other.relu1;
			linear2 = other.linear2;
			
			linear1Output = other.linear1Output;
			relu1Output = other.relu1Output;
		}
		
		return *this;
	}
	
	//! Take ownership of the given DecoderPreNetLayer.
	IdentityDecoderLayerType& operator=(IdentityDecoderLayerType&& other)
	{
		if (&other != this)
		{
			linear1 = std::move(other.linear1);
			relu1 = std::move(other.relu1);
			linear2 = std::move(other.linear2);
			
			linear1Output = std::move(other.linear1Output);
			relu1Output = std::move(other.relu1Output);
		}
		
		return *this;
	}
	
	// Forward pass.
	void Forward(const DataType& input, DataType& output)
	{
		// First linear layer.
		linear1.Forward(input, output);
	}
	
private:
	mlpack::ann::Linear linear1;
	mlpack::ann::ReLU relu1;
	mlpack::ann::Linear linear2;
	
	DataType linear1Output;
	DataType relu1Output;
};

typedef DecoderPreNetLayer<arma::mat> Decoder;
typedef IdentityDecoderLayerType<arma::mat> IdentityDecoder;

}


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
		
		for(uint32_t i = 0; i < result.words.size(); ++i){
			auto word = result.words[i];
			auto timestamp = result.wordTimeStamps[i];
			
			recognizedSegments.push_back({word, timestamp.start, timestamp.end});

		}
		
		for(auto& wordSet : recognizedSegments){
			auto [text, t0, t1] = wordSet;
						
			int samplingRate = sampling_rate;

			int64_t sampleT0 = static_cast<int64_t>(t0 * 0.001f * samplingRate);
			int64_t sampleT1 = static_cast<int64_t>(t1 * 0.001f * samplingRate);

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

//		arma::mat dataset(maxPcmSize + 2, wordData.size());
		
		arma::mat dataset(maxPcmSize + 2, wordData.size());


		// Map of words to numerical labels
		std::unordered_map<std::string, int> wordToLabel;
		
		for (auto& word : wordData) {
			std::string text = word.word;
			std::vector<float> pcm = word.pcm;
			
					
			std::vector<float> specs;
			
			librosa::feature::melspectrogram_arg melspec_arg;
			melspec_arg.y = pcm;

			std::vector<std::vector<float>> mels = librosa::feature::melspectrogram(&melspec_arg);

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
			
			for (size_t i = 0; i < maxPcmSize; ++i) {
				if (i < pcm.size()) {
					dataset(2, counter) = pcm[i];
				} else {
					dataset(2 + i, counter) = 0;
				}
			}
			
			counter++;
		}

		arma::mat labels = dataset.rows(2, dataset.n_rows - 1);
		dataset.shed_rows(2, dataset.n_rows - 1);
		
		mlModel = std::make_unique<mlpack::FFN<mlpack::MeanSquaredError>>();
		
//		mlModel->Add<viper::IdentityDecoder>(1, maxPcmSize);  // Input layer to hidden
		mlModel->Add<mlpack::Linear>(2);  // Input layer to hidden
		mlModel->Add<mlpack::ReLU>();  // Input layer to hidden
		mlModel->Add<mlpack::Linear>(maxPcmSize);  // Input layer to hidden

		// Set the model to use the stochastic gradient descent optimizer.
		ens::SGD optimizer(0.01, 512, 10000);

		mlModel->Train(dataset, labels, optimizer);
		
//
//		double stepSize = 0.001;  // Initialize step size
//		double beta1 = 0.9;  // Exponential decay rate for the first moment estimates
//		double beta2 = 0.999;  // Exponential decay rate for the weighted infinity norm estimates
//		double eps = 1e-8;  // Value used to initialize the mean and variance parameters
//		
//		// Train the model.
//		mlModel->ResetData(dataset, labels);
//		
//		size_t numEpochs = 3;
//		size_t numBatches = 4;
//
//		// Declare and initialize target and gradient matrices
//		arma::mat gradient = arma::zeros(maxPcmSize, 1);
//
//		// Training loop.
//		for (size_t epoch = 0; epoch < numEpochs; ++epoch)
//		{
//			for (size_t i = 0; i < numBatches; ++i)
//			{
//				
//				int counter = 0;
//
//				for(auto& word : wordData){
//					
//
//					std::string text = word.word;
//					std::vector<float> pcm = word.pcm;
//					
//					
//					std::vector<float> specs;
//					
//					int sr = sampling_rate;
//					int n_fft = 20;
//					int n_hop = 5;
//					std::string window = "hann";
//					bool center = false;
//					std::string pad_mode = "reflect";
//					float power = 2.f;
//					int n_mel = 40;
//					int fmin = 80;
//					int fmax = 7600;
//					
//					std::vector<std::vector<float>> mels = librosa::Feature::melspectrogram(pcm, sr, n_fft, n_hop, window, center, pad_mode, power,n_mel, fmin, fmax);
//
//					
//					for(auto& spec : mels){
//						specs.insert(specs.end(), spec.begin(), spec.end());
//					}
//
//					auto [t0, t1] = word.time;
//
//					int label = wordToLabel[text];
//
//					
//					arma::mat input(2, 1);
//
//					input(0, 0) = label;
//					input(1, 0) = std::abs(t1 - t0);
//
//					for (size_t index = 0; index < maxPcmSize; ++index) {
//						if (index < pcm.size()) {
////							input(i, 0) = specs[i];
//							labels(index, counter) = pcm[index];
//						} else {
////							input(i, 0) = 0;  // Or whatever default value makes sense
//							labels(index, counter) = 0;
//						}
//					}
//
//					arma::mat output(1, maxPcmSize);
//
//					// Forward pass.
//					mlModel->Forward(input, output);
//					
//					// Compute loss.
//					const double loss = criterion.Forward(output, labels.col(counter));
//					
//					std::cout << "Epoch: " << epoch << " Batch:  " << i << " Loss: " << loss << std::endl;
//
//					// Backward pass.
//					criterion.Backward(output, labels.col(counter), gradient);
//					
//					mlModel->Backward(input, labels.col(counter), gradient);
//					
//					counter++;
//					
//					// Update model weights.
//					optimizer.Optimize(*mlModel, gradient);
//					
//					gradient.zeros();
//
//				}
//				
//
//			}
//		}

		std::vector<std::vector<piper::Phoneme>> phonemes;
		
		piper::phonemize_eSpeak(" Para", eSpeakConfig, phonemes);

		std::vector<piper::PhonemeId> phonemeIds;
		
		std::map<piper::Phoneme, std::size_t> missingPhonemes;

		for(auto& phoneme : phonemes){
			std::vector<piper::PhonemeId> singlePhonemeIds;
			
			piper::phonemes_to_ids(phoneme, idConfig, singlePhonemeIds,
								   missingPhonemes);
			
			phonemeIds.insert(phonemeIds.end(), singlePhonemeIds.begin(), singlePhonemeIds.end());
		}

		int64_t phonemeValue = arma::accu(arma::conv_to<arma::Row<int64_t>>::from(phonemeIds));
		
		sherpa_onnx::OfflineTtsConfig config;
		config.model.vits.model = "./sharvard-medium.onnx";
		config.model.vits.tokens = "./sharvard-tokens.txt";
		config.model.vits.data_dir = "/opt/homebrew/share/espeak-ng-data";
		sherpa_onnx::OfflineTts tts(config);
		auto audio = tts.Generate(" Ven a correr", 1);

		arma::mat input(1, maxPcmSize);
		
		
		std::cout << "Sample Rate: " << audio.sample_rate << std::endl;
		
		

		float sampling_rate = 16000;
		
		float min_freq =
		std::min<int32_t>(sampling_rate, 22050);
		float lowpass_cutoff = 0.99 * 0.5 * min_freq;
		
		int32_t lowpass_filter_width = 6;
		auto resampler = std::make_unique<sherpa_onnx::LinearResample>(
														  sampling_rate, 22050, lowpass_cutoff,
														  lowpass_filter_width);
		std::vector<float> samples;
		resampler->Resample(wordData[0].pcm.data(), wordData[0].pcm.size(), true, &samples);

		librosa::stft_arg arg1;
		arg1.y = samples;
		
		auto stft1 = librosa::stft(&arg1);
		
		std::vector<std::vector<float>> magnitudeSpectrum;
		
		for (const auto &frame : stft1) {
			std::vector<float> magnitudes;
			for (const auto &complexNumber : frame) {
				float magnitude = std::sqrt(complexNumber.r * complexNumber.r + complexNumber.i * complexNumber.i);
				magnitudes.push_back(magnitude);
			}
			magnitudeSpectrum.push_back(magnitudes);
		}

		
		librosa::stft_arg arg2;
		arg2.y = audio.samples;

		auto stft2 = librosa::stft(&arg2);
		
		librosa::feature::melspectrogram_arg melspec_arg1;
		

		melspec_arg1.y = samples;

		std::vector<std::vector<float>> mels1 = librosa::feature::melspectrogram(&melspec_arg1);


		librosa::feature::melspectrogram_arg melspec_arg2;
	
		melspec_arg2.y = audio.samples;
		
		std::vector<std::vector<float>> mels2 = librosa::feature::melspectrogram(&melspec_arg2);
	
//		vector<vector<float>> adjustedMelSpec = librosa::util::nnls(mels2, mels1);

		
		librosa::feature::inverse::mel_to_stft_arg melToStftArg;
		
		melToStftArg.M = mels1;
		
		vector<vector<float>> reconstructedSTFT = librosa::feature::inverse::mel_to_stft(&melToStftArg);
		
		auto reconstructedAudio = griffinLim(reconstructedSTFT);
//
//		for(int i = 0; i<audio.samples.size(); ++i){
//			if(i >= maxPcmSize){
//				break;
//			}
//			
//			if(i < maxPcmSize){
//				input(i, 0) = audio.samples[i];
//			} else {
//				input(i, 0) = 0;
//			}
//		}

//		arma::mat output(1, maxPcmSize);
//		mlModel->Predict(input, output);

		
		// Display the decoded predictions.

		std::vector<int16_t> pcmFinal = denormalize(reconstructedAudio);

		alGenBuffers(1, &buffer);
		alBufferData(buffer, AL_FORMAT_MONO16, pcmFinal.data(), pcmFinal.size() * sizeof(int16_t), 22050 * 1);
		
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
