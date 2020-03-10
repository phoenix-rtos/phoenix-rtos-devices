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

#include "posix/utils.h"
#include "posix/idtree.h"
#include "flashsrv.h"
#include "flashdrv.h"

#include "../../../phoenix-rtos-filesystems/jffs2/libjffs2.h"

#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...) do { if (0) fprintf(stderr, __FILE__  ":%d trace: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)

extern int portGet(unsigned id);

typedef struct {
	void *next, *prev;

	int (*handler)(void *, msg_t *);
	msg_t msg;
	unsigned rid;
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
	int error;

	for (;;) {
		mutexLock(flashsrv_common.lock);
		while ((req = flashsrv_getRequest()) == NULL)
			condWait(flashsrv_common.cond, flashsrv_common.lock, 0);
		mutexUnlock(flashsrv_common.lock);

		error = req->handler(req->data, &req->msg);
		msgRespond(req->port, error, &req->msg, req->rid);
		flashsrv_freeRequest(req);
	}
}


static void flashsrv_fsThread(void *arg)
{
	flashsrv_filesystem_t *fs = arg;
	flashsrv_request_t *req;

	if (fs->mount(fs->data) < 0) {
		LOG_ERROR("mount partition %d", fs->port);
		/* TODO: cleanup */
		endthread();
	}

	for (;;) {
		req = flashsrv_newRequest(fs->port, fs->handler, fs->data);

		while (msgRecv(fs->port, &req->msg, &req->rid) < 0) {
			LOG_ERROR("msgRecv error!");
		}

		/* TODO: handle umount here? */
		if (req->msg.type == mtUmount) {

		}

		mutexLock(flashsrv_common.lock);
		flashsrv_queueRequest(req);
		mutexUnlock(flashsrv_common.lock);

		condSignal(flashsrv_common.cond);
	}
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


static int flashsrv_partoff(flashsrv_partition_t *p, off_t offset, size_t size, size_t *partoff)
{
	if (p == NULL) {
		*partoff = 0;
		return EOK;
	}

	TRACE("Partition: size %d, start %d", p->size * PAGES_PER_BLOCK * FLASH_PAGE_SIZE, p->start);
	if ((offset + size) > (p->size * PAGES_PER_BLOCK * FLASH_PAGE_SIZE)) {
		LOG_ERROR("offset out of bounds");
		return -ENXIO;
	}

	*partoff = p->start * FLASH_PAGE_SIZE * PAGES_PER_BLOCK;
	return EOK;
}


static int flashsrv_write(flashsrv_partition_t *partition, off_t offset, const char *data, size_t size)
{
	flashdrv_dma_t *dma;
	int i, err;
	char *databuf;
	void *metabuf;
	size_t partoff = 0;
	size_t writesz = size;

	if ((err = flashsrv_partoff(partition, offset, size, &partoff)) < 0)
		return err;

	offset += partoff;

	TRACE("Write off: %ld, size: %d, ptr: %p", offset, size, data);

	if (size & (FLASH_PAGE_SIZE - 1))
		return -EINVAL;

	if (offset & (FLASH_PAGE_SIZE - 1))
		return -EINVAL;

	if (data == NULL)
		return flashsrv_erase(offset, offset + size);

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;
	metabuf = flashsrv_common.metabuf;

	memset(metabuf, 0xff, sizeof(flashdrv_meta_t));

	for (i = 0; size; i++) {
		memcpy(databuf, data + FLASH_PAGE_SIZE * i, FLASH_PAGE_SIZE);
		err = flashdrv_write(dma, offset / FLASH_PAGE_SIZE + i, databuf, metabuf);

		if (err) {
			LOG_ERROR("write error %d", err);
			break;
		}
		size -= FLASH_PAGE_SIZE;
	}

	writesz -= size;

	return writesz;
}


static int flashsrv_read(flashsrv_partition_t *partition, off_t offset, char *data, size_t size)
{
	flashdrv_dma_t *dma;
	char *databuf;
	size_t rp, totalBytes = 0;
	size_t partoff = 0;
	int pageoffs, writesz, err = EOK;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;

	if ((err = flashsrv_partoff(partition, offset, size, &partoff)) < 0)
		return err;

	offset += partoff;
	rp = (offset & ~(FLASH_PAGE_SIZE - 1)) / FLASH_PAGE_SIZE;
	pageoffs = offset & (FLASH_PAGE_SIZE - 1);

	TRACE("Read off: %ld, size: %zu", offset, size);

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


static flashsrv_partition_t *flashsrv_getPartition(id_t id)
{
	flashsrv_partition_t *p;
	mutexLock(flashsrv_common.lock);
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, id));
	mutexUnlock(flashsrv_common.lock);
	return p;
}


static int flashsrv_mount(flashsrv_partition_t *partition, unsigned port, id_t *newid, mode_t *mode, const char *type, size_t len)
{
	flashsrv_filesystem_t *fs;
	int error;

	if (len > sizeof(fs->name)) {
		len = sizeof(fs->name);
	}

	if (!strncmp(type, "jffs2", len)) {
		if ((fs = calloc(1, sizeof(*fs))) == NULL) {
			LOG_ERROR("out of memory");
			return -ENOMEM;
		}

		strncpy(fs->name, type, len);

		fs->handler = jffs2lib_message_handler;
		if ((fs->port = portGet(port)) < 0) {
			LOG_ERROR("port create");
			return fs->port;
		}

		fs->data = jffs2lib_create_partition(partition->start, partition->start + partition->size, 0, fs->port, &fs->root);
		fs->mount = jffs2lib_mount_partition;

		if ((error = beginthreadex(flashsrv_fsThread, 4, fs->stack, sizeof(fs->stack), fs, &fs->tid)) < 0) {
			LOG_ERROR("beginthread");
			/* TODO: jffs2lib_destroy_partition? */
			return error;
		}

		mutexLock(flashsrv_common.lock);
		lib_rbInsert(&flashsrv_common.filesystems, &fs->node);
		mutexUnlock(flashsrv_common.lock);

		*newid = fs->root;
		*mode = S_IFDIR;
		return EOK;
	}

	LOG_ERROR("bad filesystem type: %*s", len, type);
	return -EINVAL;
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


static int flashsrv_devErase(flashsrv_partition_t *p, size_t offset, size_t size)
{
	size_t partoff = 0;
	size_t start = 0;
	size_t end = 0;
	int error;

	if ((error = flashsrv_partoff(p, offset, size, &partoff)) < 0)
		return error;

	start = offset + partoff;
	end = start + size;

	if (end % ERASE_BLOCK_SIZE || start % ERASE_BLOCK_SIZE)
		return -EINVAL;

	return flashsrv_erase(start, end);
}


#if 0      /* TODO */
static int flashsrv_devWriteRaw(flash_i_devctl_t *idevctl, const char *data)
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
#endif


static int flashsrv_devCtrl(flashsrv_partition_t *p, msg_t *msg)
{
	int error;
	unsigned long type = msg->i.devctl;
	flash_erase_t *erase = (void *)msg->i.data;

	switch (type) {
	case flashsrv_devctl_erase:
		error = flashsrv_devErase(p, erase->offset, erase->size);
		break;

	default:
		error = -EINVAL;
		break;
	}

	return error;
}


static int flashsrv_fileAttr(flashsrv_partition_t *p, int type)
{
	if (p == NULL)
		return -EOPNOTSUPP;

	switch (type) {
	case atSize:
		return p->size * FLASH_PAGE_SIZE * PAGES_PER_BLOCK;

	case atDev:
		return p->start * FLASH_PAGE_SIZE * PAGES_PER_BLOCK;

	default:
		return -EINVAL;
	}
}


static void flashsrv_devThread(void *arg)
{
	msg_t msg;
	unsigned rid, port = (unsigned)arg;
	flashsrv_partition_t *partition;
	int error;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		if ((partition = flashsrv_getPartition(msg.object)) == NULL && msg.object != ROOT_ID) {
			error = -ENODEV;
		}
		else {
			error = EOK;
			switch (msg.type) {
			case mtRead:
				if ((msg.o.io = flashsrv_read(partition, msg.i.io.offs, msg.o.data, msg.o.size)) < 0) {
					error = msg.o.io;
					msg.o.io = 0;
				}
				break;

			case mtWrite:
				if ((msg.o.io = flashsrv_write(partition, msg.i.io.offs, msg.i.data, msg.i.size)) < 0) {
					error = msg.o.io;
					msg.o.io = 0;
				}
				break;

			case mtMount:
//				LOG_ERROR("info actually: mounting: %llu\n", msg.i.mount.port);
				error = flashsrv_mount(partition, msg.i.mount.port, &msg.o.mount.id, &msg.o.mount.mode, msg.i.data, msg.i.size);
				break;

			case mtSync:
				flashsrv_syncAll();
				break;

			case mtDevCtl:
				flashsrv_devCtrl(partition, &msg);
				break;

			case mtGetAttr:
				TRACE("DEV mtgetAttr");
				*(int *)msg.o.data = flashsrv_fileAttr(partition, msg.i.attr);
				break;

			case mtOpen:
				TRACE("DEV mtOpen");
				msg.o.open = msg.object;
				/* fallthrough */
			case mtClose:
				break;

			default:
				TRACE("DEV error");
				error = -EINVAL;
				break;
			}
		}

		msgRespond(port, error, &msg, rid);
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


int main(int argc, char **argv)
{
	int i, c;
	id_t id;
	flashsrv_partition_t *p;
	rbnode_t *n;
	unsigned port;
	char path[32];

	port = PORT_DESCRIPTOR;
	lib_rbInit(&flashsrv_common.filesystems, flashsrv_fscmp, NULL);
	idtree_init(&flashsrv_common.partitions);
	flashsrv_common.queue = NULL;
	condCreate(&flashsrv_common.cond);
	mutexCreate(&flashsrv_common.lock);

	while ((c = getopt(argc, argv, "p:")) != -1) {
		switch (c) {
		case 'p':
			flashsrv_partition(atoi(argv[optind - 1]), atoi(argv[optind]));
			optind += 1;
			break;

		default:
			LOG_ERROR("imx6ull-flash: invalid argument \"%s\"", argv[optind - 1]);
			exit(EXIT_FAILURE);
			break;
		}
	}

	for (n = lib_rbMinimum(flashsrv_common.partitions.root); n; n = lib_rbNext(n)) {
		p = lib_treeof(flashsrv_partition_t, node, n);
		id = idtree_id(&p->node);

		sprintf(path, "/dev/flash%lld", id);
		create_dev(port, id, path, S_IFBLK);
	}

	create_dev(port, ROOT_ID, "/dev/flashsrv", S_IFCHR);

	if (fork())
		exit(EXIT_SUCCESS);
	setsid();

	TRACE("got port %d", port);

	flashdrv_init();
	flashsrv_common.dma = flashdrv_dmanew();
	flashsrv_common.databuf = mmap(NULL, FLASH_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_UNCACHED, -1, 0);
	flashsrv_common.rawdatabuf = mmap(NULL, 2 * FLASH_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_UNCACHED, -1, 0);
	flashsrv_common.metabuf = mmap(NULL, FLASH_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_UNCACHED, -1, 0);

	for (i = 0; i < sizeof(flashsrv_common.poolStacks) / sizeof(flashsrv_common.poolStacks[0]); ++i)
		beginthread(flashsrv_poolThread, 4, flashsrv_common.poolStacks[i], sizeof(flashsrv_common.poolStacks[i]), NULL);

	printf("imx6ull-flash: initialized\n");
	flashsrv_devThread((void *)port);
}
