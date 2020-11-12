#ifndef __DRMKPI_LOCK_H__
#define	__DRMKPI_LOCK_H__

struct ww_mutex {
	struct mutex base;
	struct cv condvar;
	struct ww_acquire_ctx *ctx;
};

int drmkpi_ww_mutex_lock_sub(struct ww_mutex *,
    struct ww_acquire_ctx *, int catch_signal);
void drmkpi_ww_mutex_unlock_sub(struct ww_mutex *);

#endif	/* __DRMKPI_LOCK_H__ */

