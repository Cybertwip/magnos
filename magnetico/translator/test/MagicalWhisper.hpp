#pragma once

#include "whisper.h"
#include <vector>
#include <string>
#include <thread>

struct WhisperTimestamp {
	int64_t start;
	int64_t end;
};

struct WhisperResult {
	std::vector<std::string> words;
	std::vector<WhisperTimestamp> timestamps;
};

class MagicalWhisper {
public:
	MagicalWhisper(const std::string& modelFilePath);
	~MagicalWhisper();
	
	WhisperResult processAudio(const std::vector<float>& pcmf32);
	
private:
	struct whisper_context* ctx = nullptr;
};
