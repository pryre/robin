#include <stdbool.h>
#include <stdint.h>

#include "drivers/drv_status_io.h"

#ifdef USE_ALSA_SOUND

#include <alsa/asoundlib.h>
#include <sndfile.h>

#define PCM_DEVICE "default"

static snd_pcm_t *pcm_handle;
static snd_pcm_hw_params_t *params;
static snd_pcm_uframes_t frames;
static int dir
static int pcmrc;

static SF_INFO sfinfo;
static SNDFILE *infile = NULL;

#endif

void status_led_arm_init( void ) {

}

void status_led_heart_init( void ) {

}

void status_buzzer_init() {
#ifdef USE_ALSA_SOUND
	char *infilename = "../documents/beep.wav";

    infile = sf_open(infilename, SFM_READ, &sfinfo);
    fprintf(stderr,"Channels: %d\n", sfinfo.channels);
    fprintf(stderr,"Sample rate: %d\n", sfinfo.samplerate);
    fprintf(stderr,"Sections: %d\n", sfinfo.sections);
    fprintf(stderr,"Format: %d\n", sfinfo.format);

    /* Open the PCM device in playback mode */
    snd_pcm_open(&pcm_handle, PCM_DEVICE, SND_PCM_STREAM_PLAYBACK, 0);

    /* Allocate parameters object and fill it with default values*/
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    /* Set parameters */
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm_handle, params, sfinfo.channels);
    snd_pcm_hw_params_set_rate(pcm_handle, params, sfinfo.samplerate, 0);

    /* Write parameters */
    snd_pcm_hw_params(pcm_handle, params);

    /* Allocate buffer to hold single period */
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    fprintf(stderr,"# frames in a period: %d\n", frames);
#endif
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
#ifdef USE_ALSA_SOUND
		int* buf = NULL;
		int readcount;

		buf = malloc(frames * sfinfo.channels * sizeof(int16_t));
		while ((readcount = sf_readf_int(infile, buf, frames))>0) {
			pcmrc = snd_pcm_writei(pcm_handle, buf, readcount);
			if (pcmrc == -EPIPE) {
				fprintf(stderr, "Underrun!\n");
				snd_pcm_prepare(pcm_handle);
			}
			else if (pcmrc < 0) {
				fprintf(stderr, "Error writing to PCM device: %s\n", snd_strerror(pcmrc));
			}
			else if (pcmrc != readcount) {
				fprintf(stderr,"PCM write difffers from PCM read.\n");
			}
		}
		fprintf(stderr,"End read/write loop\n");

		//snd_pcm_drain(pcm_handle);
		//snd_pcm_close(pcm_handle);
		free(buf);
#endif
	} else {
		//Off
	}
}
