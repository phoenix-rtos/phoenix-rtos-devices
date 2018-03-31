/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Core pseudo devices (/dev/null, /dewv/zero, /dev/random)
 *
 * Copyright 2012 Phoenix Systems
 * Author: Pawel Kolodziej
 *
 * This file is part of Phoenix-RTOS
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <main/if.h>
#include <fs/if.h>
#include <dev/if.h>

#define MINOR_NULL 0
#define MINOR_ZERO 1
#define MINOR_RANDOM 2
#define MINOR_CONSOLE 3
#define MINOR_KLOG 4

static unsigned pseudocore_seed = 0;


static int pseudocore_read(file_t* file, offs_t offs, char *buff, unsigned int len)
{
  vnode_t *vnode = file->vnode;
	int i;
	
	switch(MINOR(vnode->dev)){
		case MINOR_NULL:
			return 0;
		case MINOR_ZERO:
			memset(buff, 0, len);
			return len;
		case MINOR_RANDOM:
			for (i=0; i < len; i+=4){
				rand_r(&pseudocore_seed);
				memcpy(buff + i, &pseudocore_seed, len - i > 4 ? 4 : len - i);
			}
			return len;
		default:
			return -ENOENT;
	}

	return 0;
}


static int pseudocore_write(file_t* file, offs_t offs, char *buff, unsigned int len)
{
  vnode_t *vnode = file->vnode;
	
	switch(MINOR(vnode->dev)) {
		case MINOR_NULL:
		case MINOR_ZERO:
			break;

    case MINOR_CONSOLE:
      hal_consolePrint2(ATTR_INFO, buff, len);
      break;
    
    case MINOR_KLOG:
      if (!klog_isInitialized())
        hal_consolePrint2(ATTR_INFO, buff, len);
      else
        klog_put2(ATTR_INFO, buff, len);
      break;

    default:
			return -ENOENT;
	}
	return len;
}


static int pseudocore_truncate(vnode_t* vnode, unsigned int size)
{
    return 0;
}


void _pseudocore_init(void)
{
	static const file_ops_t pseudocore_ops = {
		.read = pseudocore_read,
		.write = pseudocore_write,
		.truncate = pseudocore_truncate,
	};

	if (dev_register(MAKEDEV(MAJOR_PSEUDO, 0), &pseudocore_ops) < 0) {
	  main_printf(ATTR_ERROR, "dev[pseudocore]: Can't register pseudo devices!\n");
	  return;
	}

/*	assert(EOK == dev_mknod(MAKEDEV(MAJOR_PSEUDO, 0), "null"));
	assert(EOK == dev_mknod(MAKEDEV(MAJOR_PSEUDO, 1), "zero"));
	assert(EOK == dev_mknod(MAKEDEV(MAJOR_PSEUDO, 2), "random"));
	assert(EOK == dev_mknod(MAKEDEV(MAJOR_PSEUDO, 3), "console"));
	assert(EOK == dev_mknod(MAKEDEV(MAJOR_PSEUDO, 4), "klog"));*/

	return;
}
