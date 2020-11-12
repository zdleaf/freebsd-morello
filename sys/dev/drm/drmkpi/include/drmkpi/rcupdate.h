#ifndef __DRMKPI_RCUPDATE_H__
#define	__DRMKPI_RCUPDATE_H__

/* BSD specific defines */
#define	RCU_TYPE_REGULAR 0
#define	RCU_TYPE_SLEEPABLE 1
#define	RCU_TYPE_MAX 2

#define	LINUX_KFREE_RCU_OFFSET_MAX	4096	/* exclusive */

struct rcu_head {
	void *raw[2];
} __aligned(sizeof(void *));

typedef void (*rcu_callback_t)(struct rcu_head *head);
typedef void (*call_rcu_func_t)(struct rcu_head *head, rcu_callback_t func);

void drmkpi_call_rcu(unsigned type, struct rcu_head *ptr, rcu_callback_t func);
void drmkpi_rcu_barrier(unsigned type);
void drmkpi_rcu_read_lock(unsigned type);
void drmkpi_rcu_read_unlock(unsigned type);
void drmkpi_synchronize_rcu(unsigned type);

#endif	/* __DRMKPI_RCUPDATE_H__ */
