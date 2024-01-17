#include "MagicalAudio.hpp"

int main(int argc, char** argv) {
	const char* filename = "./training.wav";
	MagicalAudio magicalAudio(filename);
	magicalAudio.processAudio();

	return 0;
}
