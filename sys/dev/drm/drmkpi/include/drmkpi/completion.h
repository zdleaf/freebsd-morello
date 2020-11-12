#ifndef __DRMKPI_COMPLETION_H__
#define	__DRMKPI_COMPLETION_H__

struct completion {
	unsigned int done;
};

void drmkpi_complete_common(struct completion *, int);
int drmkpi_wait_for_common(struct completion *, int);
int drmkpi_wait_for_timeout_common(struct completion *, int, int);
int drmkpi_try_wait_for_completion(struct completion *);
int drmkpi_completion_done(struct completion *);

#endif
