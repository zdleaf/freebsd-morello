#ifndef __DRMKPI_TIMER_H__
#define	__DRMKPI_TIMER_H__

struct timer_list {
	struct callout callout;
	union {
		void (*function) (unsigned long);	/* < v4.15 */
		void (*function_415) (struct timer_list *);
	};
	unsigned long data;
	int expires;
};

int drmkpi_mod_timer(struct timer_list *timer, int expires);
void drmkpi_add_timer(struct timer_list *timer);
void drmkpi_add_timer_on(struct timer_list *timer, int cpu);
int drmkpi_del_timer(struct timer_list *timer);
int drmkpi_del_timer_sync(struct timer_list *timer);

#endif /* __DRMKPI_TIMER_H__ */
