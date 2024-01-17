#include "MagicalAudio.hpp"

int main(int argc, char** argv) {
	const char* filename = "./onepiece16k.wav";
	MagicalAudio magicalAudio(filename);
	magicalAudio.processAudio();

	return 0;
}
