#ifndef __DRMKPI_WORK_H__
#define	__DRMKPI_WORK_H__

struct work_struct;
typedef void (*work_func_t)(struct work_struct *);

struct work_exec {
	TAILQ_ENTRY(work_exec) entry;
	struct work_struct *target;
};

struct workqueue_struct {
	struct taskqueue *taskqueue;
	struct mtx exec_mtx;
	TAILQ_HEAD(, work_exec) exec_head;
	atomic_t draining;
};

struct work_struct {
	struct task work_task;
	struct workqueue_struct *work_queue;
	work_func_t func;
	atomic_t state;
};

struct delayed_work {
	struct work_struct work;
	struct {
		struct callout callout;
		struct mtx mtx;
		int	expires;
	} timer;
};

extern struct workqueue_struct *drmkpi_system_wq;
extern struct workqueue_struct *drmkpi_system_long_wq;
extern struct workqueue_struct *drmkpi_system_unbound_wq;

void drmkpi_init_delayed_work(struct delayed_work *, work_func_t);
void drmkpi_work_fn(void *, int);
void drmkpi_delayed_work_fn(void *, int);
struct workqueue_struct *drmkpi_create_workqueue_common(const char *, int);
void drmkpi_destroy_workqueue(struct workqueue_struct *);
bool drmkpi_queue_work_on(int cpu, struct workqueue_struct *, struct work_struct *);
bool drmkpi_queue_delayed_work_on(int cpu, struct workqueue_struct *,
    struct delayed_work *, unsigned delay);
bool drmkpi_cancel_delayed_work(struct delayed_work *);
bool drmkpi_cancel_work_sync(struct work_struct *);
bool drmkpi_cancel_delayed_work_sync(struct delayed_work *);
bool drmkpi_flush_work(struct work_struct *);
bool drmkpi_flush_delayed_work(struct delayed_work *);
bool drmkpi_work_pending(struct work_struct *);
bool drmkpi_work_busy(struct work_struct *);
struct work_struct *drmkpi_current_work(void);

#endif
