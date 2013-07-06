#include "spinlock.h"

int __spin_lock(spinlock_t *lock)
{
	 while (__sync_lock_test_and_set(&lock->flag, 1))
		 while(lock->flag);
	 return 0;
}

void __spin_unlock(spinlock_t *lock) {
	//__sync_synchronize();
	//lock->flag=0;
	__sync_lock_release(&lock->flag);
	//(lock->flag)--;
}
