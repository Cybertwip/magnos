#include "MagicalWhisper.hpp"
#include <cstdio>
#include <cstdlib>
#include <cassert>

// command-line parameters
struct whisper_params {
	int32_t n_threads    = std::min(4, (int32_t) std::thread::hardware_concurrency());
	int32_t n_processors =  1;
	int32_t offset_t_ms  =  0;
	int32_t offset_n     =  0;
	int32_t duration_ms  =  0;
	int32_t progress_step =  5;
	int32_t max_context  = -1;
	int32_t max_len      =  0;
	int32_t best_of      = whisper_full_default_params(WHISPER_SAMPLING_GREEDY).greedy.best_of;
	int32_t beam_size    = whisper_full_default_params(WHISPER_SAMPLING_BEAM_SEARCH).beam_search.beam_size;
	
	float word_thold    =  0.01f;
	float entropy_thold =  2.40f;
	float logprob_thold = -1.00f;
	
	bool speed_up        = false;
	bool debug_mode      = false;
	bool translate       = false;
	bool detect_language = false;
	bool diarize         = false;
	bool tinydiarize     = false;
	bool split_on_word   = false;
	bool no_fallback     = false;
	bool output_txt      = false;
	bool output_vtt      = false;
	bool output_srt      = false;
	bool output_wts      = false;
	bool output_csv      = false;
	bool output_jsn      = false;
	bool output_jsn_full = false;
	bool output_lrc      = false;
	bool no_prints       = false;
	bool print_special   = false;
	bool print_colors    = false;
	bool print_progress  = false;
	bool no_timestamps   = false;
	bool log_score       = false;
	bool use_gpu         = true;
	
	std::string language  = "en";
	std::string prompt;
	std::string font_path = "/System/Library/Fonts/Supplemental/Courier New Bold.ttf";
	std::string model     = "models/ggml-base.en.bin";
	
	// [TDRZ] speaker turn string
	std::string tdrz_speaker_turn = " [SPEAKER_TURN]"; // TODO: set from command line
	
	std::string openvino_encode_device = "CPU";
	
	std::vector<std::string> fname_inp = {};
	std::vector<std::string> fname_out = {};
};

MagicalWhisper::MagicalWhisper(const std::string& modelFilePath) {

	// whisper init
	whisper_context_params cparams;
	cparams.use_gpu = true;
	ctx = whisper_init_from_file_with_params(modelFilePath.c_str(), cparams);
	
	if (ctx == nullptr) {
		fprintf(stderr, "error: failed to initialize whisper context\n");
		// Handle initialization error as needed
		exit(1);
	}
	
//	whisper_ctx_init_openvino_encoder(ctx, nullptr, params.openvino_encode_device.c_str(), nullptr);
}

MagicalWhisper::~MagicalWhisper() {
	if (ctx != nullptr) {
		whisper_free(ctx);
	}
}

WhisperResult MagicalWhisper::processAudio(const std::vector<float>& pcmf32) {
	WhisperResult result;
	
	whisper_params params;
	
	params.split_on_word = true;
	params.translate = false;
	params.detect_language = false;
	params.language = "es";
	params.max_len = 1;
//	params.beam_size = 4;
	
	whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
	
	wparams.strategy = params.beam_size > 1 ? WHISPER_SAMPLING_BEAM_SEARCH : WHISPER_SAMPLING_GREEDY;
	
	wparams.print_realtime   = false;
	wparams.print_progress   = params.print_progress;
	wparams.print_timestamps = !params.no_timestamps;
	wparams.print_special    = params.print_special;
	wparams.translate        = params.translate;
	wparams.language         = params.language.c_str();
	wparams.detect_language  = params.detect_language;
	wparams.n_threads        = params.n_threads;
	wparams.n_max_text_ctx   = params.max_context >= 0 ? params.max_context : wparams.n_max_text_ctx;
	wparams.offset_ms        = params.offset_t_ms;
	wparams.duration_ms      = params.duration_ms;
	
	wparams.token_timestamps = params.output_wts || params.output_jsn_full || params.max_len > 0;
	wparams.thold_pt         = params.word_thold;
	wparams.max_len          = params.output_wts && params.max_len == 0 ? 60 : params.max_len;
	wparams.split_on_word    = params.split_on_word;
	
	wparams.speed_up         = params.speed_up;
	wparams.debug_mode       = params.debug_mode;
	
	wparams.tdrz_enable      = params.tinydiarize; // [TDRZ]
	
	wparams.initial_prompt   = params.prompt.c_str();
	
	wparams.greedy.best_of        = params.best_of;
	wparams.beam_search.beam_size = params.beam_size;
	
	wparams.temperature_inc  = params.no_fallback ? 0.0f : wparams.temperature_inc;
	wparams.entropy_thold    = params.entropy_thold;
	wparams.logprob_thold    = params.logprob_thold;
	
	wparams.no_timestamps    = params.no_timestamps;
	
	wparams.dtw_token_timestamps = true;
	wparams.dtw_ah_preset = WHISPER_AHEADS_MEDIUM;
//	wparams.single_segment = true;
	
	if (whisper_full_parallel(ctx, wparams, const_cast<float*>(pcmf32.data()), pcmf32.size(), 1) != 0) {
		fprintf(stderr, "failed to process audio\n");
		// Handle processing error as needed
		exit(1);
	}
			
	const int n_segments = whisper_full_n_segments(ctx);
	
	bool first_token = true;
	
	int64_t initial_dtw = 0;
	
	for (int i = 0; i < n_segments; ++i) {
				
		const int64_t test_dtw0 = whisper_full_get_segment_dtw0(ctx, i);
		
		const int64_t test_dtw1 = whisper_full_get_segment_dtw1(ctx, i);
		
		const char* text = whisper_full_get_segment_text(ctx, i);

		if(test_dtw0 == -1){
			continue;
		}

		if(first_token){
			initial_dtw = whisper_full_get_segment_t0(ctx, i);
			first_token = false;
		}

		int64_t t0 = 0;
		int64_t t1 = 0;
		
		t0 = test_dtw0 * 10;
		t1 = test_dtw1 * 10;
		
		if(test_dtw1 == -1){
			t1 = t0;
		}
		
		if(!result.timestamps.empty()){
			if(t0 == t1){
				t0 = result.timestamps.back().end;
				t1 = t0 + std::abs(t0 - (test_dtw0 * 10));
			}
		} else {
			if(t0 == t1){
				t0 = 0;
			}
		}
		
		result.words.push_back(text);
		result.timestamps.push_back({t0, t1});
	}
	
	return result;
}
