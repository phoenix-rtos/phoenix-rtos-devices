#include "phoenix_list.h"
#include <stdatomic.h>

/*
	Note that this is minimal implementation for RCU variants
	that is sufficient for Hailo driver use-case. Might not work
	in other enviroments.
*/

void INIT_LIST_HEAD(struct list_head *list)
{
	list->prev = list;
	list->next = list;
}


void list_add(struct list_head *new, struct list_head *head)
{
	head->next = new;
	new->next = head->next->next;
	new->prev = head;
	head->prev->next = new;
}

void list_add_rcu(struct list_head *new, struct list_head *head)
{
	list_add(new, head);
	/* Make sure to flush writes */
	atomic_thread_fence(memory_order_release);
}


void list_del(struct list_head *entry)
{
	if (entry->next == entry) {
		/* List is empty - corrupted state */
		entry->next = NULL;
		entry->prev = NULL;
		return;
	}
	/* Remove from the list */
	entry->prev->next = entry->next;
	entry->next->prev = entry->prev;
}


void list_del_rcu(struct list_head *entry)
{
	list_del(entry);
	/* Make sure to flush all writes */
	atomic_thread_fence(memory_order_release);
}

void rcu_read_lock(void)
{
	return;
}

void rcu_read_unlock(void)
{
	return;
}

void synchronize_rcu(void)
{
	atomic_thread_fence(memory_order_release);
}
