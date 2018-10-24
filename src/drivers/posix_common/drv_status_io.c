#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_status_io.h"

/*
#ifdef USE_ALSA_SOUND
#include "drivers/posix_common/beeper.h"

snd_pcm_t* audio_sink_handle;
wavedata_t beepFile;
#endif
*/
void status_led_arm_init( void ) {

}

void status_led_heart_init( void ) {

}

void status_buzzer_init() {
/*
#ifdef USE_ALSA_SOUND
	// Configure Output Device
	audio_sink_handle = Audio_openDevice();

	Audio_readWaveFileIntoMemory(AUDIO_FILE_BEEP, &beepFile);

	// Cleanup, letting the music in buffer play out (drain), then close and free.
	//snd_pcm_drain(handle);
	//snd_pcm_hw_free(handle);
	//snd_pcm_close(handle);
	//free(sampleFile.pData);
#endif
*/
}

void status_led_arm_set( bool on ) {
	if( on ) {
		//On
	} else {
		//Off
	}
}

void status_led_heart_set( bool on ) {
	if( on ) {
		//On
	} else {
		//Off
	}
}

void status_buzzer_set( bool on ) {
	if( on ) {
	/*
#ifdef USE_ALSA_SOUND
	Audio_playFile(audio_sink_handle, &beepFile);
#endif
	*/
	} else {
		//Off
	}
}
