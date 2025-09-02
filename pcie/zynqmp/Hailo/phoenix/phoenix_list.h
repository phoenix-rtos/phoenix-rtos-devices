#ifndef PHOENIX_LIST
#define PHOENIX_LIST

#include <stddef.h>

struct list_head {
	struct list_head *prev, *next;
};

/*
	To create linked list of any structure and any layout all
	we need to do is add list_head structure inside it.
	Now when we will iterate over this list we will only
	operate using list_head field in this structure. This
	pointer might be placed anywhere in the struct so how
	do we retrive other fields?

	MAGIC OF contianer_of MACRO !!!

	How does it work?
	First given type of struct that our list_head struct is
	contained in (and over which we lets say iterate) we establish
	what is the offset of field member relative to list_head that
	ptr points to.

	typeof( ((type *)0)->member ) - retrives type of member within
	the struct type.

	... *__mptr = (ptr); - this will assing value of ptr to helper
	so that we will not modify value provided by the user in.

	(char *)__mptr - offsetof(type,member) - remove offset of member
	from __mptr, and now we have pointer pointing to beggining
	of the struct.

	(type *)... - performs simple cast.
*/


#define container_of(ptr, type, member) ({ (type *)((char *)ptr - offsetof(type, member)); })

#define list_first_entry(ptr, type, member) \
	container_of((ptr)->next, type, member)

#define list_next_entry(pos, member) \
	container_of((pos)->member.next, typeof(*(pos)), member)

#define list_for_each_entry(pos, head, member) \
	for (pos = list_first_entry(head, typeof(*pos), member); \
			&pos->member != (head); \
			pos = list_next_entry(pos, member))

#define list_for_each_entry_safe(pos, n, head, member) \
	for (pos = list_first_entry(head, typeof(*pos), member), \
		n = list_next_entry(pos, member); \
			&pos->member != (head); \
			pos = n, n = list_next_entry(n, member))

#define list_for_each_entry_rcu(pos, head, member) list_for_each_entry(pos, head, member)

#define list_empty(head) (head.next == head)

#define list_is_last(head, node) (node.next == head)


void INIT_LIST_HEAD(struct list_head *list);
void list_add(struct list_head *new, struct list_head *head);
void list_add_rcu(struct list_head *new, struct list_head *head);
void list_del(struct list_head *entry);
void list_del_rcu(struct list_head *entry);
void rcu_read_lock(void);
void rcu_read_unlock(void);
void synchronize_rcu(void);

#endif
