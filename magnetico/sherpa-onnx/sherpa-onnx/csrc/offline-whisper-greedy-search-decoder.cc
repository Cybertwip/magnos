// sherpa-onnx/csrc/offline-whisper-greedy-search-decoder.cc
//
// Copyright (c)  2023  Xiaomi Corporation

#include "sherpa-onnx/csrc/offline-whisper-greedy-search-decoder.h"

#include <algorithm>
#include <utility>
#include <random>

#include "sherpa-onnx/csrc/macros.h"
#include "sherpa-onnx/csrc/onnx-utils.h"

namespace sherpa_onnx {


class CustomDiscreteDistribution {
public:
	CustomDiscreteDistribution(const std::vector<float>& probabilities)
	: probabilities_(probabilities.begin(), probabilities.end()) {}
	
	int operator()(std::default_random_engine& engine) {
		std::discrete_distribution<int> dist(probabilities_.begin(), probabilities_.end());
		return dist(engine);
	}
	
private:
	std::vector<int> probabilities_;
};


struct whisper_token_data {
	int id;
	int tid;
	float p;
	float plog;
	float pt;
	float ptsum;
	int placeholder1; // Add relevant type
	int placeholder2; // Add relevant type
	float placeholder3; // Add relevant type
};

whisper_token_data whisper_sample_token(
										const std::vector<float>& probs,
										const std::vector<float>& logprobs,
										int timestamp_begin,
										int eot,
										bool best
										) {
	whisper_token_data result = {
		0, 0, 0.0f, 0.0f, 0.0f, 0.0f, -1, -1, 0.0f
	};
	
	const int n_logits = probs.size();
	
	// Calculate sum and max for timestamp tokens
	{
		float sum_ts = 0.0;
		float max_ts = 0.0;
		
		for (int i = timestamp_begin; i < n_logits; i++) {
			if (probs[i] == -INFINITY) {
				continue;
			}
			
			sum_ts += probs[i];
			if (max_ts < probs[i]) {
				max_ts = probs[i];
				result.tid = i;
			}
		}
		
		result.pt    = max_ts / (sum_ts + 1e-10);
		result.ptsum = sum_ts;
	}
	
	if (best) {
		// Best sampling
		for (int i = 0; i < n_logits; ++i) {
			if (result.p < probs[i]) {
				result.id   = i;
				result.p    = static_cast<float>(probs[i]);
				result.plog = static_cast<float>(logprobs[i]);
			}
		}
	} else {
		// Random sampling
		CustomDiscreteDistribution dist(probs);
		std::default_random_engine engine; // Add a relevant random engine

		result.id   = dist(engine);
		result.p    = static_cast<float>(probs[result.id]);
		result.plog = static_cast<float>(logprobs[result.id]);
	}
	
	if (result.id >= timestamp_begin && result.id < eot) {
		result.tid = result.id;
		result.pt  = result.p;
	}
	
	return result;
}

void maskTimestamps(std::vector<int64_t>& tokens, std::vector<float>& logits_vector, int timestamp_begin, int eot) {
	int n_samples = tokens.size();
	int n_logits = logits_vector.size();
	
	for (int k = 0; k < n_samples; ++k) {
		int token_val = tokens[k];
		
		// Adjust indices to account for flattened vector
		bool last_was_timestamp = k >= 1 && tokens[k - 1] >= timestamp_begin;
		bool penultimate_was_timestamp = k < 2 || tokens[k - 2] >= timestamp_begin;
		
		if (last_was_timestamp) {
			if (penultimate_was_timestamp) {
				// has to be non-timestamp
				std::fill(logits_vector.begin() + timestamp_begin, logits_vector.end(), -std::numeric_limits<float>::infinity());
			} else {
				// cannot be normal text tokens
				std::fill(logits_vector.begin(), logits_vector.begin() + eot, -std::numeric_limits<float>::infinity());
			}
		}
	}
}

// Assuming logits_vector is a 1D array or vector and max_token_id is the index of the selected token
float sampleTimestampLogProb(const std::vector<float>& logits_vector, std::vector<float>& logprobs, int max_token_id, int timestamp_begin) {
	int n_logits = logits_vector.size();
	
	// Assuming logits_vector contains log probabilities (logits) for each token
	const float logit_max = *std::max_element(logits_vector.begin(), logits_vector.end());
	
	for (int i = 0; i < n_logits; ++i) {
		if (logits_vector[i] > -std::numeric_limits<float>::infinity()) {
			logprobs[i] = logits_vector[i] - logit_max;
		} else {
			logprobs[i] = -std::numeric_limits<float>::infinity();
		}
	}
	
	float sampled_prob = logprobs[max_token_id];
	return sampled_prob;
}
std::pair<std::vector<float>, std::vector<float>> logsoftmax(std::vector<float>& logits, int32_t beg_tok){
	
	int32_t n_logits = logits.size();
	
	std::vector<float> logprobs(n_logits);
	std::vector<float> probs(n_logits);
	
	
	
	// populate the logprobs array (log_softmax)
	{
		const float logit_max = *std::max_element(logits.begin(), logits.end());
		float logsumexp = 0.0f;
		for (int i = 0; i < n_logits; ++i) {
			if (logits[i] > -INFINITY) {
				logsumexp += expf(logits[i] - logit_max);
			}
		}
		logsumexp = logf(logsumexp) + logit_max;
		
		for (int i = 0; i < n_logits; ++i) {
			if (logits[i] > -INFINITY) {
				logprobs[i] = logits[i] - logsumexp;
			} else {
				logprobs[i] = -INFINITY;
			}
		}
	}
	
	// if sum of probability over timestamps is above any other token, sample timestamp
	// ref: https://github.com/openai/whisper/blob/0b1ba3d46ebf7fe6f953acfd8cad62a4f851b49f/whisper/decoding.py#L431-L437
	{
		// logsumexp over timestamps
		float timestamp_logprob = -INFINITY;
		{
			float logsumexp = 0.0f;
			const float logprob_max = *std::max_element(logprobs.begin() + beg_tok, logprobs.end());
			for (int i = beg_tok; i < n_logits; ++i) {
				if (logprobs[i] > -INFINITY) {
					logsumexp += expf(logprobs[i] - logprob_max);
				}
			}
			if (logsumexp > 0.0f) {
				timestamp_logprob = logf(logsumexp) + logprob_max;
			}
		}
		
		const float max_text_token_logprob = *std::max_element(logprobs.begin(), logprobs.begin() + beg_tok);
		
		if (timestamp_logprob > max_text_token_logprob) {
			for (int i = 0; i < beg_tok; ++i) {
				logits[i]   = -INFINITY;
				logprobs[i] = -INFINITY;
			}
		}
	}
	
	// compute probs
	{
		for (int i = 0; i < n_logits; ++i) {
			if (logits[i] == -INFINITY) {
				probs[i] = 0.0f;
			} else {
				probs[i] = expf(logprobs[i]);
			}
		}
	}
	
	return {probs, logprobs};

}

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


std::vector<float> OfflineWhisperGreedySearchDecoder::DetectTimeStamps(std::vector<int64_t> initial_tokens, Ort::Value &cross_k, Ort::Value &cross_v) const
{  
	initial_tokens.clear();
	
	initial_tokens.push_back(model_->SOT());
	
//	initial_tokens.push_back(model_->TimeStampsBeginToken());
	
	auto memory_info =
		Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);

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
	
	cross_k = std::move(std::get<3>(decoder_out));
	cross_v = std::move(std::get<4>(decoder_out));

	*(std::get<5>(decoder_out).GetTensorMutableData<int64_t>()) =
	initial_tokens.size();
	
	const auto &logits = std::get<0>(decoder_out);
	const float *p_logits = logits.GetTensorData<float>();
	
	auto logits_shape = logits.GetTensorTypeAndShapeInfo().GetShape();
	int32_t vocab_size = logits_shape[2];
	
	const float *p_start = p_logits + (logits_shape[1] - 1) * vocab_size;
	
	int32_t max_token_id = static_cast<int32_t>(
												std::distance(p_start, std::max_element(p_start, p_start + vocab_size)));
	int32_t n_text_ctx = model_->TextCtx();
	
	bool in_timestamps = false;
	std::vector<float> timestamps;
	
	for (int32_t i = 0; i < n_text_ctx; ++i) {
		if (max_token_id == model_->EOT()) {
			break;
		}
		
		std::array<int64_t, 2> token_shape{1, 1};
		Ort::Value tokens = Ort::Value::CreateTensor<int64_t>(
															  model_->Allocator(), token_shape.data(), token_shape.size());
		
		auto tokens_vector = tensor_to_vec<int64_t>(tokens);
		
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
		auto logits_vector = tensor_to_vec<float>(logits);
		
		// Get the actual token value based on max_token_id
		
		
		if (max_token_id >= model_->TimeStampsBeginToken()) {
			float this_logit = logits_vector[max_token_id];
			timestamps.push_back(this_logit);
		}
		
		// Handle the actual token value (actual_token) as needed
		
		// Check if the next token is the EndTimeStampsToken
		if (max_token_id == model_->NoTimeStampsToken()) {
			// End of timestamps
			in_timestamps = false;
		}
		
		// Use actual_token or perform additional processing based on your needs
		
		// Update max_token_id for the next iteration
		max_token_id = static_cast<int64_t>(
											std::distance(logits_vector.begin(),
														  std::max_element(logits_vector.begin(),
																		   logits_vector.begin() + vocab_size)));
	}

	return timestamps;
	

//	
//	const int offset_samples = (16000*1)/1000;
//	const int n_samples_per_processor = (30 * 16000 - offset_samples)/1;
//
//	const int64_t offset_t = (int64_t) 1/10.0;
//
//	int64_t t0 += 100 * ((i + 1) * n_samples_per_processor) / 16000 + offset_t;
//	int64_t t1 += 100 * ((i + 1) * n_samples_per_processor) / 16000 + offset_t;
//	
//	

//	auto t0 = 0 + 2*(tokens_cur.front().tid - model_->TimeStampsBeginToken());
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
	
	
	auto timestamps = DetectTimeStamps(initial_tokens, cross_k, cross_v);
	
  	initial_tokens.push_back(model_->NoTimeStampsToken());
	
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
	
	const auto &logits = std::get<0>(decoder_out);
	const float *p_logits = logits.GetTensorData<float>();
	
	auto logits_shape = logits.GetTensorTypeAndShapeInfo().GetShape();
	int32_t vocab_size = logits_shape[2];
	
	const float *p_start = p_logits + (logits_shape[1] - 1) * vocab_size;
	
	int32_t max_token_id = static_cast<int32_t>(
												std::distance(p_start, std::max_element(p_start, p_start + vocab_size)));
	
	int32_t n_text_ctx = model_->TextCtx();
	
	std::vector<int32_t> predicted_tokens;
	for (int32_t i = 0; i < n_text_ctx; ++i) {
		if (max_token_id == model_->EOT()) {
			break;
		}
		
		predicted_tokens.push_back(max_token_id);
		
		std::array<int64_t, 2> token_shape{1, 1};
		Ort::Value tokens = Ort::Value::CreateTensor<int64_t>(
															  model_->Allocator(), token_shape.data(), token_shape.size());
		
		int64_t *p_tokens = tokens.GetTensorMutableData<int64_t>();
		p_tokens[0] = max_token_id;
		
		decoder_out = model_->ForwardDecoder(std::move(tokens),
											 std::move(std::get<1>(decoder_out)),
											 std::move(std::get<2>(decoder_out)),
											 std::move(std::get<3>(decoder_out)),
											 std::move(std::get<4>(decoder_out)),
											 std::move(std::get<5>(decoder_out)));
		
		int64_t *p_offset =
		std::get<5>(decoder_out).GetTensorMutableData<int64_t>();
		
		*p_offset += 1;
		
		const auto &logits = std::get<0>(decoder_out);
		const float *p_logits = logits.GetTensorData<float>();
		
		max_token_id = static_cast<int64_t>(std::distance(p_logits, std::max_element(p_logits, p_logits + vocab_size)));
	}
	
	std::vector<OfflineWhisperDecoderResult> ans(1);
	
	ans[0].tokens = std::move(predicted_tokens);
	ans[0].timestamps = std::move(timestamps);

	return ans;
}

}  // namespace sherpa_onnx
