#include "drivers/posix_common/beeper.h"

#ifdef USE_ALSA_SOUND
/*
 *  Small program to read a 16-bit, signed, 44.1kHz wave file and play it.
 *  Written by Brian Fraser, heavily based on code found at:
 *  http://www.alsa-project.org/alsa-doc/alsa-lib/_2test_2pcm_min_8c-example.html
 */

#include <alsa/asoundlib.h>

// Open the PCM audio output device and configure it.
// Returns a handle to the PCM device; needed for other actions.
snd_pcm_t *Audio_openDevice() {
	snd_pcm_t *handle;

	// Open the PCM output
	int err = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
	if (err < 0) {
		printf("Play-back open error: %s\n", snd_strerror(err));
		exit(EXIT_FAILURE);
	}

	// Configure parameters of PCM output
	err = snd_pcm_set_params(handle,
			SND_PCM_FORMAT_S16_LE,
			SND_PCM_ACCESS_RW_INTERLEAVED,
			NUM_CHANNELS,
			SAMPLE_RATE,
			1,			// Allow software resampling
			50000);		// 0.05 seconds per buffer
	if (err < 0) {
		printf("Play-back configuration error: %s\n", snd_strerror(err));
		exit(EXIT_FAILURE);
	}

	return handle;
}

// Read in the file to dynamically allocated memory.
// !! Client code must free memory in wavedata_t !!
void Audio_readWaveFileIntoMemory(char *fileName, wavedata_t *pWaveStruct)
{
	assert(pWaveStruct);

	// Wave file has 44 bytes of header data. This code assumes file
	// is correct format.
	const int DATA_OFFSET_INTO_WAVE = 44;

	// Open file
	FILE *file = fopen(fileName, "r");
	if (file == NULL) {
		fprintf(stderr, "ERROR: Unable to open file %s.\n", fileName);
		exit(EXIT_FAILURE);
	}

	// Get file size
	fseek(file, 0, SEEK_END);
	int sizeInBytes = ftell(file) - DATA_OFFSET_INTO_WAVE;
	fseek(file, DATA_OFFSET_INTO_WAVE, SEEK_SET);
	pWaveStruct->numSamples = sizeInBytes / SAMPLE_SIZE;

	// Allocate Space
	pWaveStruct->pData = malloc(sizeInBytes);
	if (pWaveStruct->pData == NULL) {
		fprintf(stderr, "ERROR: Unable to allocate %d bytes for file %s.\n",
				sizeInBytes, fileName);
		exit(EXIT_FAILURE);
	}

	// Read data:
	int samplesRead = fread(pWaveStruct->pData, SAMPLE_SIZE, pWaveStruct->numSamples, file);
	if (samplesRead != pWaveStruct->numSamples) {
		fprintf(stderr, "ERROR: Unable to read %d samples from file %s (read %d).\n",
				pWaveStruct->numSamples, fileName, samplesRead);
		exit(EXIT_FAILURE);
	}

	fclose(file);
}

// Play the audio file (blocking)
void Audio_playFile( snd_pcm_t *handle, wavedata_t *pWaveData ) {
	// If anything is waiting to be written to screen, can be delayed unless flushed.
	fflush(stdout);

	// Write data and play sound (blocking)
	snd_pcm_sframes_t frames = snd_pcm_writei(handle, pWaveData->pData, pWaveData->numSamples);

	// Check for errors
	if (frames < 0)
		frames = snd_pcm_recover(handle, frames, 0);

	if (frames < 0) {
		fprintf(stderr, "ERROR: Failed writing audio with snd_pcm_writei(): %li\n", frames);
		exit(EXIT_FAILURE);
	}

	if (frames > 0 && frames < pWaveData->numSamples)
		printf("Short write (expected %d, wrote %li)\n", pWaveData->numSamples, frames);
}
#endif
