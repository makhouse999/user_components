#ifndef __USER_ES8311_H__
#define __USER_ES8311_H__

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>

#include <sys/queue.h>

struct track_params {
	const char * name;
	const uint8_t * pcm_start;
	const uint8_t * pcm_end;
	uint8_t blk_fg;

	SLIST_ENTRY(track_params) next;
};

struct es8311_dev_desc {
	struct track_params * track_handle;
	size_t track_nbr;
};


int es8311_play(char * name);
int user_es8311_init(struct es8311_dev_desc * desc);

#endif