#ifndef __DRMKPI_MUTEX_H__
#define	__DRMKPI_MUTEX_H__

typedef struct mutex {
	struct sx sx;
} mutex_t;

int drmkpi_mutex_lock_interruptible(mutex_t *m);

#endif
