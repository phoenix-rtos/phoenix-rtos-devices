#include <sys/msg.h>
#include <sys/minmax.h>
#include <sys/list.h>
#include <sys/rb.h>
#include <sys/threads.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <stdlib.h>
#include <syslog.h>

#include "posix/utils.h"
#include "posix/idtree.h"
#include "imx6ull-flashsrv.h"
#include "flashdrv.h"

#include "../../../phoenix-rtos-filesystems/jffs2/libjffs2.h"

#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...) do { if (0) fprintf(stderr, __FILE__  ":%d trace: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)

typedef struct {
	void *next, *prev;

	int (*handler)(void *, msg_t *);
	msg_t msg;
	unsigned long rid;
	unsigned port;
	void *data;
} flashsrv_request_t;


typedef struct {
	rbnode_t node;

	unsigned port;
	long root;
	int (*handler)(void *, msg_t *);
	int (*mount)(void *);
	void *data;
	unsigned tid;
	char stack[4 * 4096] __attribute__((aligned(8)));
	char name[16];
} flashsrv_filesystem_t;


typedef struct {
	idnode_t node;
	size_t start;
	size_t size;
} flashsrv_partition_t;


struct {
	char poolStacks[4][4 * 4096] __attribute__((aligned(8)));

	rbtree_t filesystems;
	idtree_t partitions;
	flashsrv_request_t *queue;
	handle_t lock, cond;

	flashdrv_dma_t *dma;
	void *databuf;
	void *rawdatabuf;
	void *metabuf;
} flashsrv_common;


static flashsrv_request_t *flashsrv_newRequest(unsigned port, int (*handler)(void *, msg_t *), void *data)
{
	flashsrv_request_t *req;

	if ((req = calloc(1, sizeof(*req))) != NULL) {
		req->port = port;
		req->handler = handler;
		req->data = data;
	}

	return req;
}


static int flashsrv_fscmp(rbnode_t *n1, rbnode_t *n2)
{
	flashsrv_filesystem_t *f1 = lib_treeof(flashsrv_filesystem_t, node, n1);
	flashsrv_filesystem_t *f2 = lib_treeof(flashsrv_filesystem_t, node, n2);

	if (f1->port < f2->port)
		return -1;
	else if (f1->port > f2->port)
		return 1;
	else
		return 0;
}


static void flashsrv_freeRequest(flashsrv_request_t *req)
{
	free(req);
}


static void flashsrv_queueRequest(flashsrv_request_t *req)
{
	LIST_ADD(&flashsrv_common.queue, req);
}


static flashsrv_request_t *flashsrv_getRequest(void)
{
	flashsrv_request_t *req;

	if ((req = flashsrv_common.queue) != NULL)
		LIST_REMOVE(&flashsrv_common.queue, req);

	return req;
}


static void flashsrv_poolThread(void *arg)
{
	flashsrv_request_t *req;

	for (;;) {
		mutexLock(flashsrv_common.lock);
		while ((req = flashsrv_getRequest()) == NULL)
			condWait(flashsrv_common.cond, flashsrv_common.lock, 0);
		mutexUnlock(flashsrv_common.lock);

		req->handler(req->data, &req->msg);

		msgRespond(req->port, &req->msg, req->rid);
		flashsrv_freeRequest(req);
	}
}


static void flashsrv_fsThread(void *arg)
{
	flashsrv_filesystem_t *fs = arg;
	flashsrv_request_t *req;
	int logmask;

	/* turn off logging to avoid possible deadlock */
	logmask = setlogmask(0x80000000);

	if (fs->mount(fs->data) < 0) {
		LOG_ERROR("mount partition %d", fs->port);
		setlogmask(logmask);
		/* TODO: cleanup */
		endthread();
	}
	setlogmask(logmask ? logmask : 0xffffffff);

	for (;;) {
		req = flashsrv_newRequest(fs->port, fs->handler, fs->data);

		if (msgRecv(fs->port, &req->msg, &req->rid) < 0)
			continue;

		/* TODO: handle umount here? */
		if (req->msg.type == mtUmount) {

		}

		mutexLock(flashsrv_common.lock);
		flashsrv_queueRequest(req);
		mutexUnlock(flashsrv_common.lock);

		condSignal(flashsrv_common.cond);
	}
}


static flashsrv_filesystem_t *flashsrv_mountFs(flashsrv_partition_t *partition, unsigned mode, char *name)
{
	flashsrv_filesystem_t *fs;

	TRACE("mountFs called");

	if ((fs = calloc(1, sizeof(*fs))) == NULL) {
		LOG_ERROR("out of memory");
		return NULL;
	}

	strncpy(fs->name, name, sizeof(fs->name));
	fs->name[sizeof(fs->name) - 1] ='\0';

	if (!strcmp(fs->name, "jffs2")) {
		fs->handler = jffs2lib_message_handler;
		portCreate(&fs->port);
		TRACE("creating jffs2 partition at port %d", fs->port);
		fs->data = jffs2lib_create_partition(partition->start, partition->start + partition->size, mode, fs->port, &fs->root);
		fs->mount = jffs2lib_mount_partition;

		if (beginthreadex(flashsrv_fsThread, 4, fs->stack, sizeof(fs->stack), fs, &fs->tid) < 0) {
			LOG_ERROR("beginthread");
			/* TODO: cleanup */
			return NULL;
		}

		mutexLock(flashsrv_common.lock);
		lib_rbInsert(&flashsrv_common.filesystems, &fs->node);
		mutexUnlock(flashsrv_common.lock);

		return fs;
	}

	LOG_ERROR("bad fs type");
	return NULL;
}


static int flashsrv_erase(size_t start, size_t end)
{
	flashdrv_dma_t *dma;
	int i, err;

	TRACE("Erase %d %d", start, end);

	if (start % (FLASH_PAGE_SIZE * PAGES_PER_BLOCK))
		return -EINVAL;

	if (end % (FLASH_PAGE_SIZE * PAGES_PER_BLOCK))
		return -EINVAL;

	if (end < start)
		return -EINVAL;

	start /= FLASH_PAGE_SIZE * PAGES_PER_BLOCK;
	end /= FLASH_PAGE_SIZE * PAGES_PER_BLOCK;

	dma = flashsrv_common.dma;

	for (i = start; i < end; i++) {
		err = flashdrv_erase(dma, i * PAGES_PER_BLOCK);

		if (err) {
			LOG_ERROR("erase error %d", err);
			break;
		}
	}

	return err;
}


static int flashsrv_partoff(id_t id, size_t start, size_t size, size_t *partoff)
{
	flashsrv_partition_t *p = NULL;
	id_t rootID = ROOT_ID;

	if (id == rootID) {
		*partoff = 0;
		return EOK;
	}

	mutexLock(flashsrv_common.lock);
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, id));
	mutexUnlock(flashsrv_common.lock);

	if (p == NULL)
		return -EINVAL;

	TRACE("Partition: size %d, start %d", p->size * PAGES_PER_BLOCK * FLASH_PAGE_SIZE, p->start);
	if ((start + size) > (p->size * PAGES_PER_BLOCK * FLASH_PAGE_SIZE))
		return -EINVAL;

	*partoff = p->start * FLASH_PAGE_SIZE * PAGES_PER_BLOCK;

	return EOK;
}


static int flashsrv_write(id_t id, size_t start, char *data, size_t size)
{
	flashdrv_dma_t *dma;
	int i, err;
	char *databuf;
	void *metabuf;
	size_t partoff = 0;
	size_t writesz = size;

	if (flashsrv_partoff(id, start, size, &partoff) < 0)
		return -EINVAL;

	start += partoff;

	TRACE("Write off: %d, size: %d, ptr: %p", start, size, data);

	if (size & (FLASH_PAGE_SIZE - 1))
		return -EINVAL;

	if (start & (FLASH_PAGE_SIZE - 1))
		return -EINVAL;

	if (data == NULL)
		return flashsrv_erase(start, start + size);

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;
	metabuf = flashsrv_common.metabuf;

	memset(metabuf, 0xff, sizeof(flashdrv_meta_t));

	for (i = 0; size; i++) {
		memcpy(databuf, data + FLASH_PAGE_SIZE * i, FLASH_PAGE_SIZE);
		err = flashdrv_write(dma, start / FLASH_PAGE_SIZE + i, databuf, metabuf);

		if (err) {
			LOG_ERROR("write error %d", err);
			break;
		}
		size -= FLASH_PAGE_SIZE;
	}

	writesz -= size;

	return writesz;
}


static int flashsrv_read(id_t id, size_t offset, char *data, size_t size)
{
	flashdrv_dma_t *dma;
	char *databuf;
	size_t rp, totalBytes = 0;
	size_t partoff = 0;
	int pageoffs, writesz, err = EOK;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;

	if (flashsrv_partoff(id, offset, size, &partoff) < 0)
		return -EINVAL;

	offset += partoff;
	rp = (offset & ~(FLASH_PAGE_SIZE - 1)) / FLASH_PAGE_SIZE;
	pageoffs = offset & (FLASH_PAGE_SIZE - 1);

	TRACE("Read off: %d, size: %d.", offset, size);

	while (size) {
		err = flashdrv_read(dma, rp, databuf, flashsrv_common.metabuf);

		if (err == flash_uncorrectable) {
			LOG_ERROR("uncorrectable read");
			err = -EIO;
			break;
		}

		writesz = min(size, FLASH_PAGE_SIZE - pageoffs);
		memcpy(data + totalBytes, databuf, writesz);

		size -= writesz;
		totalBytes += writesz;
		rp++;

		pageoffs = 0;
	}

	return totalBytes;
}


static int flashsrv_mount(mount_msg_t *mnt, oid_t *oid)
{
	flashsrv_filesystem_t *fs;
	flashsrv_partition_t *p;
	TRACE("mount received");

	mutexLock(flashsrv_common.lock);
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, mnt->id));
	mutexUnlock(flashsrv_common.lock);

	if (p == NULL) {
		LOG_ERROR("partition %ld not found", mnt->id);
		return -ENOENT;
	}

	fs = flashsrv_mountFs(p, mnt->mode, mnt->fstype);

	if (fs != NULL) {
		oid->port = fs->port;
		oid->id = fs->root;
	}
	else {
		LOG_ERROR("mount failed");
		oid->port = -1;
		oid->id = 0;
	}

	return EOK;
}


static void flashsrv_syncAll(void)
{
	msg_t msg = {0};
	rbnode_t *n;
	flashsrv_filesystem_t *fs;

	msg.type = mtSync;

	for (n = lib_rbMinimum(flashsrv_common.filesystems.root); n; n = lib_rbNext(n)) {
		fs = lib_treeof(flashsrv_filesystem_t, node, n);
		msgSend(fs->port, &msg);
	}
}


static int flashsrv_devErase(flash_i_devctl_t *idevctl, int type)
{
	size_t partoff = 0;
	size_t start = 0;
	size_t end = 0;

	if ( type == flashsrv_devctl_erase) {
		if (flashsrv_partoff(idevctl->erase.oid.id, idevctl->erase.offset, idevctl->erase.size, &partoff) < 0)
			return -EINVAL;
	}

	start = idevctl->erase.offset + partoff;
	end = start + idevctl->erase.size;

	if (end % ERASE_BLOCK_SIZE || start % ERASE_BLOCK_SIZE)
		return -EINVAL;

	return flashsrv_erase(start, end);
}


static int flashsrv_devWriteRaw(flash_i_devctl_t *idevctl, char *data)
{
	flashdrv_dma_t *dma;
	int i, err;
	char *databuf;
	size_t size = idevctl->write.size;
	size_t writesz = idevctl->write.size;

	TRACE("RAW write off: %d, size: %d, ptr: %p", idevctl->write.address, idevctl->write.size, data);

	if (size % RAW_FLASH_PAGE_SIZE)
		return -EINVAL;

	if (idevctl->write.address % RAW_FLASH_PAGE_SIZE)
		return -EINVAL;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;

	for (i = 0; size; i++) {
		memcpy(databuf, data + RAW_FLASH_PAGE_SIZE * i, RAW_FLASH_PAGE_SIZE);
		err = flashdrv_writeraw(dma, idevctl->write.address / RAW_FLASH_PAGE_SIZE + i, databuf, RAW_FLASH_PAGE_SIZE);

		if (err) {
			LOG_ERROR("raw write error %d", err);
			break;
		}
		size -= RAW_FLASH_PAGE_SIZE;
	}
	writesz -= size;

	return writesz;
}


static int flashsrv_devWriteMeta(flash_i_devctl_t *idevctl, char* data)
{
	flashdrv_dma_t *dma;
	int i, err;
	char *databuf;
	size_t size = idevctl->write.size;
	size_t writesz = idevctl->write.size;

	if (size & (FLASH_PAGE_SIZE - 1))
		return -EINVAL;

	if (idevctl->write.address & (FLASH_PAGE_SIZE - 1))
		return -EINVAL;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;

	memcpy(databuf, data, FLASH_PAGE_SIZE);
	for (i = 0; size; i++) {
		err = flashdrv_write(dma, idevctl->write.address / FLASH_PAGE_SIZE + i, NULL, databuf);

		if (err) {
			LOG_ERROR("write error %d", err);
			break;
		}
		size -= FLASH_PAGE_SIZE;
	}

	writesz -= size;

	return writesz;
}


static int flashsrv_devReadRaw(flash_i_devctl_t *idevctl, char *data)
{
	flashdrv_dma_t *dma;
	char *databuf;
	size_t rp, totalBytes = 0;
	size_t size = idevctl->readraw.size;
	size_t offset = idevctl->readraw.address;
	int err = EOK;

	if ( (size % RAW_FLASH_PAGE_SIZE) || (offset % RAW_FLASH_PAGE_SIZE) )
		return -EINVAL;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.rawdatabuf;
	rp = offset / RAW_FLASH_PAGE_SIZE;

	while (size) {
		err = flashdrv_readraw(dma, rp, databuf, RAW_FLASH_PAGE_SIZE);
		memcpy(data, databuf, RAW_FLASH_PAGE_SIZE);

		if (err == flash_uncorrectable) {
			LOG_ERROR("uncorrectable read");
			err = -EIO;
			break;
		}

		size -= RAW_FLASH_PAGE_SIZE;
		totalBytes += RAW_FLASH_PAGE_SIZE;
		rp++;
	}

	return totalBytes;
}


static void flashsrv_devCtrl(msg_t *msg)
{
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	switch (idevctl->type) {
	case flashsrv_devctl_erase :
		odevctl->err = flashsrv_devErase(idevctl, flashsrv_devctl_erase);
		break;

	case flashsrv_devctl_chiperase :
		odevctl->err = flashsrv_devErase(idevctl, flashsrv_devctl_chiperase);
		break;

	case flashsrv_devctl_writeraw :
		odevctl->err = flashsrv_devWriteRaw(idevctl, msg->i.data);
		break;

	case flashsrv_devctl_writemeta :
		odevctl->err = flashsrv_devWriteMeta(idevctl, msg->i.data);
		break;

	case flashsrv_devctl_readraw :
		odevctl->err = flashsrv_devReadRaw(idevctl, msg->o.data);
		break;

	default:
		odevctl->err = -EINVAL;
		break;
	}
}


static int flashsrv_fileAttr(int type, id_t id)
{
	flashsrv_partition_t *p = NULL;

	mutexLock(flashsrv_common.lock);
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, id));
	mutexUnlock(flashsrv_common.lock);

	if (p == NULL)
		return -1;

	switch (type) {
	case atSize:
		return p->size * FLASH_PAGE_SIZE * PAGES_PER_BLOCK;

	case atDev:
		return p->start * FLASH_PAGE_SIZE * PAGES_PER_BLOCK;

	default:
		return -1;
	}
}


static void flashsrv_devThread(void *arg)
{
	msg_t msg;
	unsigned long rid;
	unsigned port = (unsigned)arg;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtRead:
			TRACE("DEV read - id: %llu, size: %d, off: %llu ", msg.i.io.oid.id, msg.o.size, msg.i.io.offs);
			msg.o.io.err = flashsrv_read(msg.i.io.oid.id, msg.i.io.offs, msg.o.data, msg.o.size);
			break;

		case mtWrite:
			TRACE("DEV write - id: %llu, size: %d, off: %llu", msg.i.io.oid.id, msg.i.size, msg.i.io.offs);
			msg.o.io.err = flashsrv_write(msg.i.io.oid.id, msg.i.io.offs, msg.i.data, msg.i.size ? msg.i.size : msg.i.io.len);
			break;

		case mtMount:
			flashsrv_mount((mount_msg_t *)msg.i.raw, (oid_t *)msg.o.raw);
			break;

		case mtSync:
			flashsrv_syncAll();
			break;

		case mtDevCtl:
			flashsrv_devCtrl(&msg);
			break;

		case mtGetAttr:
			TRACE("DEV mtgetAttr");
			msg.o.attr.val = flashsrv_fileAttr(msg.i.attr.type, msg.i.attr.oid.id);
			break;

		case mtOpen:
			TRACE("DEV mtOpen");
		case mtClose:
			msg.o.io.err = EOK;
			break;

		default:
			TRACE("DEV error");
			msg.o.io.err = -EINVAL;
			break;
		}

		msgRespond(port, &msg, rid);
	}
}


static int flashsrv_partition(size_t start, size_t size)
{
	flashsrv_partition_t *p;

	p = malloc(sizeof(*p));

	p->start = start;
	p->size = size;

	mutexLock(flashsrv_common.lock);
	idtree_alloc(&flashsrv_common.partitions, &p->node);
	TRACE("partition allocated, start: %u, a:t id %d", start, idtree_id(&p->node));
	mutexUnlock(flashsrv_common.lock);

	return EOK;
}


static void daemonize(void)
{
	/* TODO: required when we are not root */
}


int main(int argc, char **argv)
{
	int i, c;
	oid_t oid = {0, 0}, rootoid;
	flashsrv_filesystem_t *rootfs = NULL;
	flashsrv_partition_t *p;
	rbnode_t *n;
	unsigned port;
	char path[32];

	portCreate(&port);

	TRACE("got port %d", port);

	condCreate(&flashsrv_common.cond);
	mutexCreate(&flashsrv_common.lock);
	lib_rbInit(&flashsrv_common.filesystems, flashsrv_fscmp, NULL);
	idtree_init(&flashsrv_common.partitions);

	flashsrv_common.queue = NULL;

	flashdrv_init();
	flashsrv_common.dma = flashdrv_dmanew();
	flashsrv_common.databuf = mmap(NULL, FLASH_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, NULL, -1);
	flashsrv_common.rawdatabuf = mmap(NULL, 2 * FLASH_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, NULL, -1);
	flashsrv_common.metabuf = mmap(NULL, FLASH_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, NULL, -1);

	for (i = 0; i < sizeof(flashsrv_common.poolStacks) / sizeof(flashsrv_common.poolStacks[0]); ++i)
		beginthread(flashsrv_poolThread, 4, flashsrv_common.poolStacks[i], sizeof(flashsrv_common.poolStacks[i]), NULL);

	while ((c = getopt(argc, argv, "r:p:")) != -1) {
		switch (c) {
		case 'r':
			if (argv[optind] == NULL) {
				LOG_ERROR("invalid number of arguments");
				return -1;
			}
			p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, atoi(argv[optind])));

			if (p == NULL) {
				LOG_ERROR("partition %d not found", atoi(argv[optind]));
				return -1;
			}

			rootfs = flashsrv_mountFs(p, 0, argv[optind - 1]);
			optind += 1;

			if (rootfs == NULL) {
				LOG_ERROR("failed to mount root partition");
				return -1;
			}

			rootoid.port = rootfs->port;
			rootoid.id = rootfs->root;
			break;

		case 'p':
			if (argv[optind] == NULL) {
				LOG_ERROR("invalid number of arguments");
				return -1;
			}
			flashsrv_partition(atoi(argv[optind - 1]), atoi(argv[optind]));
			optind += 1;
			break;

		default:
			break;
		}
	}

	for (n = lib_rbMinimum(flashsrv_common.partitions.root); n; n = lib_rbNext(n)) {
		p = lib_treeof(flashsrv_partition_t, node, n);
		oid.id = idtree_id(&p->node);
		oid.port = port;

		sprintf(path, "flash%lld", oid.id);
		create_dev(&oid, path);
	}

	oid.port = port;
	oid.id = ROOT_ID;

	create_dev(&oid, "flashsrv");
	if (rootfs == NULL)
		daemonize();
	else
		portRegister(rootfs->port, "/", &rootoid);

	printf("imx6ull-flash: initialized\n");
	flashsrv_devThread((void *)port);
}
