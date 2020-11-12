#ifndef __DRMKPI_MUTEX_H__
#define	__DRMKPI_MUTEX_H__

typedef struct mutex {
	struct sx sx;
} mutex_t;

#endif
