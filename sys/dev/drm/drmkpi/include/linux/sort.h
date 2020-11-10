#ifndef __DRMKPI_SORT_H__
#define	__DRMKPI_SORT_H__

#include <linux/types.h>

void sort(void *base, size_t num, size_t size,
	  int (*cmp)(const void *, const void *),
	  void (*swap)(void *, void *, int));

#endif	/* __DRMKPI_SORT_H__ */
