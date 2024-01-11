// sherpa-onnx/csrc/offline-whisper-greedy-search-decoder.cc
//
// Copyright (c)  2023  Xiaomi Corporation

#include "sherpa-onnx/csrc/offline-whisper-greedy-search-decoder.h"

#include <algorithm>
#include <utility>
#include <random>
#include <iostream>
#include <deque>

#include <vector>
#include <numeric>

#include "sherpa-onnx/csrc/macros.h"
#include "sherpa-onnx/csrc/onnx-utils.h"

namespace sherpa_onnx {


// Function to find peaks in a 1D array
template <typename T>
std::vector<int> findPeaks(const std::vector<T>& data, T threshold, int minDistance) {
	std::vector<int> peaks;
	
	for (int i = 1; i < data.size() - 1; ++i) {
		if (data[i] > threshold && data[i] > data[i - 1] && data[i] > data[i + 1]) {
			peaks.push_back(i);
			i += minDistance; // Skip the next minDistance elements to avoid nearby peaks
		}
	}
	
	return peaks;
}

static const float WHISPER_NUMBER_OF_SAMPLES = 30;
static const float SHERPA_SAMPLING_RATE = 16000;
static const float WHISPER_HOP_LENGTH = 160;
static const float AUDIO_SAMPLES_PER_TOKEN = WHISPER_HOP_LENGTH * 2;
static const float AUDIO_TIME_PER_TOKEN = AUDIO_SAMPLES_PER_TOKEN / SHERPA_SAMPLING_RATE;

template <typename T>
std::vector<T> tensor_to_vec(const Ort::Value& tensor){
	return std::vector(tensor.GetTensorData<T>(), tensor.GetTensorData<T>() + tensor.GetTensorTypeAndShapeInfo().GetElementCount());
}

int32_t OfflineWhisperGreedySearchDecoder::DetectLanguage(
														  Ort::Value &cross_k, Ort::Value &cross_v) const
{  // NOLINT
	int64_t token_val = model_->SOT();
	std::array<int64_t, 2> token_shape{1, 1};
	
	auto memory_info =
	Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
	
	Ort::Value tokens = Ort::Value::CreateTensor(
												 memory_info, &token_val, 1, token_shape.data(), token_shape.size());
	
	auto self_kv_cache = model_->GetInitialSelfKVCache();
	
	std::array<int64_t, 1> offset_shape{1};
	Ort::Value offset = Ort::Value::CreateTensor<int64_t>(
														  model_->Allocator(), offset_shape.data(), offset_shape.size());
	*(offset.GetTensorMutableData<int64_t>()) = 0;
	
	auto decoder_out = model_->ForwardDecoder(
											  std::move(tokens), std::move(self_kv_cache.first),
											  std::move(self_kv_cache.second), std::move(cross_k), std::move(cross_v),
											  std::move(offset));
	
	cross_k = std::move(std::get<3>(decoder_out));
	cross_v = std::move(std::get<4>(decoder_out));
	
	const float *p_logits = std::get<0>(decoder_out).GetTensorData<float>();
	int32_t vocab_size = model_->VocabSize();
	const auto &all_language_ids = model_->GetAllLanguageIDs();
	
	int32_t lang_id = all_language_ids[0];
	float this_logit = p_logits[lang_id];
	
	for (int32_t i = 1; i != all_language_ids.size(); ++i) {
		int32_t id = all_language_ids[i];
		float p = p_logits[id];
		
		if (p > this_logit) {
			this_logit = p;
			lang_id = id;
		}
	}
#if 1
	SHERPA_ONNX_LOGE("Detected language: %s",
					 model_->GetID2Lang().at(lang_id).c_str());
#endif
	
	return lang_id;
}


std::vector<float> OfflineWhisperGreedySearchDecoder::DetectTimeStamps( std::vector<int32_t> decoded_tokens) const
{
	
	std::deque<float> timestamps;
	
	float t_beg = 0;
	
	float total_distance = 0;
	
	std::deque<float> distances;
	
	
	for(auto max_token_id : decoded_tokens){
		float distance = max_token_id;
		total_distance += distance;
		
		distances.push_back(distance);
	}
	
	bool initial = true;
	
	t_beg = distances[1] - distances[0];
	
	for(auto distance : distances){
		
		float tt = t_beg + distance;
		
		t_beg = tt;

		float target_percentage = tt / total_distance;
		
		tt *= AUDIO_TIME_PER_TOKEN;
		
//		const int n_samples_per_processor = WHISPER_NUMBER_OF_SAMPLES * SHERPA_SAMPLING_RATE;
		
//		tt = tt * n_samples_per_processor;
		
		int32_t tsms = (100ll*tt)/SHERPA_SAMPLING_RATE;
		
		timestamps.push_back(tsms);
	}
	
	// shift timestamps
	//	timestamps.pop_front();
	
	return std::vector<float>(timestamps.begin(), timestamps.end());
}



std::vector<float> OfflineWhisperGreedySearchDecoder::DetectTimeStamps(std::vector<float> cross_k, std::vector<float> cross_v, std::vector<int32_t> decoded_tokens) const
{
	// Calculate attention scores
	std::vector<float> attentionScores(cross_k.size());
	std::transform(cross_k.begin(), cross_k.end(), cross_v.begin(), attentionScores.begin(), std::multiplies<float>());
	
	// Smooth the attention scores (optional, for better peak detection)
	std::vector<float> smoothedScores(attentionScores.size());
	for (int i = 0; i < attentionScores.size(); ++i) {
		smoothedScores[i] = std::accumulate(attentionScores.begin() + std::max(0, i - 2),
											attentionScores.begin() + std::min(static_cast<int>(attentionScores.size()), i + 3),
											0.0f) / 5.0f;  // Simple moving average (adjust as needed)
	}
	
	// Find peaks in attention scores
	float threshold = 0.5f;  // Adjust as needed
	int minDistance = 10;    // Adjust as needed
	std::vector<int> peaks;
	
	auto decoded_peaks = findPeaks(smoothedScores, threshold, minDistance);
	size_t counter = 0;
	for(int32_t decoded_token : decoded_tokens){
		peaks.push_back(decoded_peaks[counter++]);
	}
	
	// Analyze peaks to estimate word boundaries
	std::vector<int> wordBoundaries = peaks;
	std::vector<int> wordLengths;
	for (size_t i = 0; i < wordBoundaries.size() - 1; ++i) {
		wordLengths.push_back(wordBoundaries[i + 1] - wordBoundaries[i]);
	}
	
	std::cout << "Estimated word boundaries: ";
	for (int boundary : wordBoundaries) {
		std::cout << boundary << " ";
	}
	std::cout << "\nEstimated word lengths: ";
	for (int length : wordLengths) {
		std::cout << length << " ";
	}
	std::cout << std::endl;
	
	return {};
	
	
}

std::vector<OfflineWhisperDecoderResult>
OfflineWhisperGreedySearchDecoder::Decode(Ort::Value cross_k,
										  Ort::Value cross_v) {
	auto memory_info =
	Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
	
	// For multilingual models, initial_tokens contains [sot, language, task]
	//   - language is English by default
	//   - task is transcribe by default
	//
	// For non-multilingual models, initial_tokens contains [sot]
	std::vector<int64_t> initial_tokens = model_->GetInitialTokens();
	
	if (model_->IsMultiLingual()) {
		if (!config_.language.empty()) {
			const auto &lang2id = model_->GetLang2ID();
			
			if (!lang2id.count(config_.language)) {
				SHERPA_ONNX_LOGE("Invalid language: %s", config_.language.c_str());
				exit(-1);
			}
			
			int32_t lang_id = lang2id.at(config_.language);
			
			// 0: sot, 1: lang_id, 2: task, 3: no_timestamps
			initial_tokens[1] = lang_id;
		} else {
			int32_t lang_id = DetectLanguage(cross_k, cross_v);
			
			// 0: sot, 1: lang_id, 2: task, 3: no_timestamps
			initial_tokens[1] = lang_id;
		}
		
		if (config_.task == "translate") {
			initial_tokens[2] = model_->Translate();
		} else if (config_.task != "transcribe") {
			// initial_tokens[2] is transcribe by default
			SHERPA_ONNX_LOGE(
							 "Unsupported task: %s. Valid values are: transcribe, translate.",
							 config_.task.c_str());
		}
	}
	
//	initial_tokens.push_back(model_->TimeStampsBeginToken());
	
	int32_t batch_size = 1;
	std::array<int64_t, 2> token_shape{
		batch_size, static_cast<int64_t>(initial_tokens.size())};
	
	Ort::Value tokens = Ort::Value::CreateTensor(
												 memory_info, initial_tokens.data(), initial_tokens.size(),
												 token_shape.data(), token_shape.size());
	
	std::array<int64_t, 1> offset_shape{1};
	Ort::Value offset = Ort::Value::CreateTensor<int64_t>(
														  model_->Allocator(), offset_shape.data(), offset_shape.size());
	*(offset.GetTensorMutableData<int64_t>()) = 0;
	
	auto self_kv_cache = model_->GetInitialSelfKVCache();
	
	auto decoder_out = model_->ForwardDecoder(
											  std::move(tokens), std::move(self_kv_cache.first),
											  std::move(self_kv_cache.second), std::move(cross_k), std::move(cross_v),
											  std::move(offset));
	
	*(std::get<5>(decoder_out).GetTensorMutableData<int64_t>()) =
	initial_tokens.size();
	
	cross_k = std::move(std::get<3>(decoder_out));
	cross_v = std::move(std::get<4>(decoder_out));
	
	const auto &logits = std::get<0>(decoder_out);
	const float *p_logits = logits.GetTensorData<float>();
	
	auto logits_shape = logits.GetTensorTypeAndShapeInfo().GetShape();
	int32_t vocab_size = logits_shape[2];
	
	const float *p_start = p_logits + (logits_shape[1] - 1) * vocab_size;
	
	// Calculate softmax to get probabilities
	std::vector<float> probabilities(vocab_size);
	for (int i = 0; i < vocab_size; ++i) {
		probabilities[i] = std::exp(p_logits[i]);
	}
	float sum = std::accumulate(probabilities.begin(), probabilities.end(), 0.0);
	for (int i = 0; i < vocab_size; ++i) {
		probabilities[i] /= sum;
	}
	
	// Calculate log probabilities
	std::vector<float> log_probs(vocab_size);
	for (int i = 0; i < vocab_size; ++i) {
		log_probs[i] = std::log(probabilities[i]);
	}
	const float *p_log_probs =
	log_probs.data();
	
	int32_t num_frames = static_cast<int32_t>(logits_shape[1]);
	
	
	int32_t max_token_id = static_cast<int32_t>(
												std::distance(p_start, std::max_element(p_start, p_start + vocab_size)));
	

	auto max_token_tt = static_cast<int64_t>(std::distance(
													  static_cast<const float *>(p_log_probs),
													  std::max_element(
																	   static_cast<const float *>(p_log_probs),
																	   static_cast<const float *>(p_log_probs) + vocab_size)));
	p_log_probs += 1;

	int32_t n_text_ctx = model_->TextCtx();
	int blank_id_ = 0;
	int prev_id = -1;
	
	std::vector<int32_t> predicted_tokens;
	std::vector<int32_t> predicted_timestamps;
	std::vector<float> timestamps;
	float distance = 0.0f;
	
	
	for (int32_t i = 0; i < n_text_ctx; ++i) {
		if (max_token_id == model_->EOT()) {
			break;
		}
		
		if(max_token_id != model_->TimeStampsBeginToken()){
			predicted_tokens.push_back(max_token_id);
		}
		
		if (max_token_tt != blank_id_ && max_token_tt != prev_id && max_token_tt != model_->TimeStampsBeginToken()) {
			int64_t time = max_token_tt;
			std::cout << "Id: " << max_token_tt << std::endl;
			std::cout << "Time: " << time << std::endl;
			
			distance += time;
			
			predicted_timestamps.push_back(time);
						
			std::cout << "Distance: " << distance << std::endl;
			
			prev_id = max_token_tt;

		}

		std::array<int64_t, 2> token_shape{1, 1};
		Ort::Value tokens = Ort::Value::CreateTensor<int64_t>(
															  model_->Allocator(), token_shape.data(), token_shape.size());
		
		int64_t *p_tokens = tokens.GetTensorMutableData<int64_t>();
		p_tokens[0] = max_token_id;
		
		decoder_out = model_->ForwardDecoder(std::move(tokens),
											 std::move(std::get<1>(decoder_out)),
											 std::move(std::get<2>(decoder_out)),
											 std::move(cross_k),
											 std::move(cross_v),
											 std::move(std::get<5>(decoder_out)));
		
		
		cross_k = std::move(std::get<3>(decoder_out));
		cross_v = std::move(std::get<4>(decoder_out));
		
		int64_t *p_offset =
		std::get<5>(decoder_out).GetTensorMutableData<int64_t>();
		
		*p_offset += 1;
		
		const auto &logits = std::get<0>(decoder_out);
		const float *p_logits = logits.GetTensorData<float>();
		
		max_token_id = static_cast<int64_t>(std::distance(p_logits, std::max_element(p_logits, p_logits + vocab_size)));
	
	
		max_token_tt = static_cast<int64_t>(std::distance(
														   static_cast<const float *>(p_log_probs),
														   std::max_element(
																			static_cast<const float *>(p_log_probs),
																			static_cast<const float *>(p_log_probs) + vocab_size)));

		p_log_probs += 1;

		
	}
	
	auto cross_k_vector = tensor_to_vec<float>(cross_k);
	auto cross_v_vector = tensor_to_vec<float>(cross_v);
	
	//	auto _ = DetectTimeStamps(cross_k_vector, cross_v_vector, predicted_tokens);
	//	auto timestamps = DetectTimeStamps(predicted_tokens);
	
	
	std::vector<OfflineWhisperDecoderResult> ans(1);
	
	timestamps = DetectTimeStamps(predicted_timestamps);

	ans[0].tokens = std::move(predicted_tokens);
	ans[0].timestamps = std::move(timestamps);
	
	
	return ans;
}

}  // namespace sherpa_onnx
