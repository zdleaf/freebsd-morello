#ifndef __DRMKPI_SRCU_H__
#define	__DRMKPI_SRCU_H__

struct srcu_struct {
};

int drmkpi_srcu_read_lock(struct srcu_struct *);
void drmkpi_srcu_read_unlock(struct srcu_struct *, int index);
void drmkpi_synchronize_srcu(struct srcu_struct *);
void drmkpi_srcu_barrier(struct srcu_struct *);
int drmkpi_init_srcu_struct(struct srcu_struct *);
void drmkpi_cleanup_srcu_struct(struct srcu_struct *);

#endif	/* __DRMKPI_SRCU_H__ */
