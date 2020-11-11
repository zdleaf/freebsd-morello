#ifndef __LINUX_SORT_H__
#define	__LINUX_SORT_H__

#include <linux/types.h>
#include <drmkpi/sort.h>

#define	sort(b, n, s, cmp, swap)	drmkpi_sort(b, n, s, cmp, swap)

#endif	/* __LINUX_SORT_H__ */
