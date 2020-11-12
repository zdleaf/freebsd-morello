#ifndef __DRMKPI_SCHED_H__
#define	__DRMKPI_SCHED_H__

#include <drmkpi/completion.h>

#define	MAX_SCHEDULE_TIMEOUT	INT_MAX

#define	TASK_RUNNING		0x0000
#define	TASK_INTERRUPTIBLE	0x0001
#define	TASK_UNINTERRUPTIBLE	0x0002
#define	TASK_NORMAL		(TASK_INTERRUPTIBLE | TASK_UNINTERRUPTIBLE)
#define	TASK_WAKING		0x0100
#define	TASK_PARKED		0x0200

#define	TASK_COMM_LEN		(MAXCOMLEN + 1)

struct work_struct;
struct task_struct {
	struct thread *task_thread;
	struct mm_struct *mm;
	linux_task_fn_t *task_fn;
	void   *task_data;
	int	task_ret;
	atomic_t usage;
	atomic_t state;
	atomic_t kthread_flags;
	pid_t	pid;	/* BSD thread ID */
	const char    *comm;
	void   *bsd_ioctl_data;
	unsigned bsd_ioctl_len;
	struct completion parked;
	struct completion exited;
#define	TS_RCU_TYPE_MAX 2
	TAILQ_ENTRY(task_struct) rcu_entry[TS_RCU_TYPE_MAX];
	int rcu_recurse[TS_RCU_TYPE_MAX];
	int bsd_interrupt_value;
	struct work_struct *work;	/* current work struct, if set */
	struct task_struct *group_leader;
};

#define	current	({ \
	struct thread *__td = curthread; \
	linux_set_current(__td); \
	((struct task_struct *)__td->td_lkpi_task); \
})

static inline void
linux_schedule_save_interrupt_value(struct task_struct *task, int value)
{
	task->bsd_interrupt_value = value;
}

static inline int
linux_schedule_get_interrupt_value(struct task_struct *task)
{
	int value = task->bsd_interrupt_value;
	task->bsd_interrupt_value = 0;
	return (value);
}

bool drmkpi_signal_pending(struct task_struct *task);
bool drmkpi_fatal_signal_pending(struct task_struct *task);
bool drmkpi_signal_pending_state(long state, struct task_struct *task);
void drmkpi_send_sig(int signo, struct task_struct *task);

int drmkpi_schedule_timeout(int timeout);

#endif	/* __DRMKPI_SCHED_H__ */
