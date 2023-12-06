#ifndef __USER_UTILS_H__
#define __USER_UTILS_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/queue.h>

enum nvs_dat_ty{
	TY_STR,
	TY_BLOB,
};
	
struct nvs_user_zone {
	char name[32];
	enum nvs_dat_ty ty;
};

struct nvs_user_key {
	struct nvs_user_zone * handle;
	char name[32];
	SLIST_ENTRY(nvs_user_key) next;
};



uint32_t get_time(void);
int set_time(uint32_t ts);

int str_trim_space(char * in, char * out);

char * get_nvs(char * key, void * pbuf, size_t * len);
int set_nvs(char * key, void * pbuf, size_t len);
int clear_nvs(char * key);

int create_nvs_key(const char * name, enum nvs_dat_ty ty);

int user_utils_init(void);

#endif

