#ifndef __DRMKPI_WAIT_H__
#define	__DRMKPI_WAIT_H__

struct wait_queue;
struct wait_queue_head;

#define	wait_queue_entry wait_queue

typedef struct wait_queue wait_queue_t;
typedef struct wait_queue_entry wait_queue_entry_t;
typedef struct wait_queue_head wait_queue_head_t;

typedef int wait_queue_func_t(wait_queue_t *, unsigned int, int, void *);

/*
 * Many API consumers directly reference these fields and those of
 * wait_queue_head.
 */
struct wait_queue {
	unsigned int flags;	/* always 0 */
	void *private;
	wait_queue_func_t *func;
	union {
		struct list_head task_list; /* < v4.13 */
		struct list_head entry; /* >= v4.13 */
	};
};

struct wait_queue_head {
	spinlock_t lock;
	union {
		struct list_head task_list; /* < v4.13 */
		struct list_head head; /* >= v4.13 */
	};
};

/*
 * This function is referenced by at least one DRM driver, so it may not be
 * renamed and furthermore must be the default wait queue callback.
 */
extern wait_queue_func_t drmkpi_autoremove_wake_function;
extern wait_queue_func_t drmkpi_default_wake_function;

void drmkpi_wake_up_bit(void *, int);
int drmkpi_wait_on_bit_timeout(unsigned long *, int, unsigned int, int);
void drmkpi_wake_up_atomic_t(atomic_t *);
int drmkpi_wait_on_atomic_t(atomic_t *, unsigned int);

void drmkpi_init_wait_entry(wait_queue_t *, int);
void drmkpi_wake_up(wait_queue_head_t *, unsigned int, int, bool);

int drmkpi_wait_event_common(wait_queue_head_t *, wait_queue_t *, int,
    unsigned int, spinlock_t *);

bool drmkpi_waitqueue_active(wait_queue_head_t *);

void drmkpi_prepare_to_wait(wait_queue_head_t *, wait_queue_t *, int);
void drmkpi_finish_wait(wait_queue_head_t *, wait_queue_t *);

struct task_struct;
bool drmkpi_wake_up_state(struct task_struct *, unsigned int);

#endif	/* __DRMKPI_WAIT_H__ */
