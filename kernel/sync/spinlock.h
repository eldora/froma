#ifndef SYNC_H
#define SYNC_H

typedef volatile struct spinlock_t {
	  int flag;
}spinlock_t;

int __spin_lock(spinlock_t *lock);
void __spin_unlock(spinlock_t *lock);

#endif
