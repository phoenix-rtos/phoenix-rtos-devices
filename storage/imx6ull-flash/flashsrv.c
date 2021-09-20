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

#include <posix/utils.h>
#include <posix/idtree.h>
#include "imx6ull-flashsrv.h"
#include "imx6ull-flashdrv.h"

#include <libjffs2.h>

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
	size_t start; /* in erase blocks */
	size_t size;  /* in erase blocks */
} flashsrv_partition_t;


struct {
	char poolStacks[4][4 * 4096] __attribute__((aligned(8)));

	rbtree_t filesystems;
	idtree_t partitions;
	flashsrv_request_t *queue;
	handle_t lock, cond;

	flashdrv_dma_t *dma;
	flashsrv_info_t info;

	void *databuf; /* at least writesz + metasz */
	void *metabuf; /* at least metasz */
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
	int err, erased = 0;
	const unsigned int pages_per_block = flashsrv_common.info.erasesz / flashsrv_common.info.writesz;
	unsigned int blockno;

	TRACE("Erase %zu %zu", start, end);

	if (start % flashsrv_common.info.erasesz)
		return -EINVAL;

	if (end % flashsrv_common.info.erasesz)
		return -EINVAL;

	if (end < start)
		return -EINVAL;

	start /= flashsrv_common.info.erasesz;
	end /= flashsrv_common.info.erasesz;

	dma = flashsrv_common.dma;

	for (blockno = start; blockno < end; blockno++) {
		uint32_t paddr = blockno * pages_per_block;
		if (flashdrv_isbad(dma, paddr)) {
			LOG_ERROR("erase: skipping bad block: %u", blockno);
			continue;
		}

		if ((err = flashdrv_erase(dma, paddr)) < 0) {
			LOG_ERROR("error while erasing block: %d - marking as badblock", blockno);
			if ((err = flashdrv_markbad(dma, paddr)) < 0)
				return -1; /* can't mark as badblock, urecoverable error */
		}
		else {
			erased += 1;
		}
	}

	return erased;
}


static int flashsrv_partoff(id_t id, int raw, size_t start, size_t size, size_t *partoff, size_t *partsize)
{
	flashsrv_partition_t *p = NULL;
	unsigned int erase_block_size = flashsrv_common.info.erasesz;

	if (raw) /* addresses in RAW page sizes */
		erase_block_size = (flashsrv_common.info.erasesz / flashsrv_common.info.writesz) * (flashsrv_common.info.writesz + flashsrv_common.info.metasz);

	mutexLock(flashsrv_common.lock);
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, id));
	mutexUnlock(flashsrv_common.lock);

	if (p == NULL)
		return -ENOENT;

	TRACE("Partition: size %u, start %u (in EB)", p->size, p->start);
	if ((start + size) > (p->size * erase_block_size))
		return -EINVAL;

	*partoff = p->start * erase_block_size;

	if (partsize)
		*partsize = p->size * erase_block_size;

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

	if (flashsrv_partoff(id, 0, start, size, &partoff, NULL) < 0)
		return -EINVAL;

	start += partoff;

	TRACE("Write off: %d, size: %d, ptr: %p", start, size, data);

	if (size % flashsrv_common.info.writesz)
		return -EINVAL;

	if (start % flashsrv_common.info.writesz)
		return -EINVAL;

	if (data == NULL)
		return flashsrv_erase(start, start + size);

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;
	metabuf = flashsrv_common.metabuf;

	/* FIXME: this breaks meta parity bits, we need to read meta before writing data? */
	memset(metabuf, 0xff, sizeof(flashdrv_meta_t));

	for (i = 0; size; i++) {
		/* TODO: should we skip badblocks ? */
		memcpy(databuf, data + flashsrv_common.info.writesz * i, flashsrv_common.info.writesz);
		err = flashdrv_write(dma, start / flashsrv_common.info.writesz + i, databuf, metabuf);

		if (err) {
			LOG_ERROR("write error %d", err);
			break;
		}
		size -= flashsrv_common.info.writesz;
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

	if ((err = flashsrv_partoff(id, 0, offset, size, &partoff, NULL)) < 0)
		return err;

	offset += partoff;
	rp = offset / flashsrv_common.info.writesz;
	pageoffs = offset % flashsrv_common.info.writesz;

	TRACE("Read off: %d, size: %d.", offset, size);

	while (size) {
		err = flashdrv_read(dma, rp, databuf, flashsrv_common.metabuf);

		/* FIXME: read errors are being returned in meta->errors[] */
		/* TODO: rewrite if correctable error? */
		if (err == flash_uncorrectable) {
			LOG_ERROR("uncorrectable read");
			err = -EIO;
			break;
		}


		writesz = min(size, flashsrv_common.info.writesz - pageoffs);
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


static int flashsrv_devErase(const flash_i_devctl_t *idevctl)
{
	size_t start = idevctl->erase.address;
	size_t size = idevctl->erase.size;
	size_t partoff, partsize;
	int err;

	TRACE("DevErase: off:%zu, size: %zu", start, size);

	if ((err = flashsrv_partoff(idevctl->erase.oid.id, 0, start, size, &partoff, &partsize)) < 0)
		return err;

	if (size == 0)
		size = partsize - start;
	start += partoff;

	return flashsrv_erase(start, start + size);
}


static int flashsrv_devWriteRaw(flash_i_devctl_t *idevctl, char *data)
{
	flashdrv_dma_t *dma;
	int i, err;
	char *databuf;
	const unsigned int raw_page_size = flashsrv_common.info.writesz + flashsrv_common.info.metasz;
	size_t partoff;
	size_t size = idevctl->write.size;
	size_t writesz = idevctl->write.size;
	size_t start = idevctl->write.address;

	TRACE("RAW write off: %d, size: %d, ptr: %p", start, size, data);

	if (size % raw_page_size)
		return -EINVAL;

	if (start % raw_page_size)
		return -EINVAL;

	if ((err = flashsrv_partoff(idevctl->write.oid.id, 1, start, size, &partoff, NULL)) < 0) /* raw = 1 */
		return err;

	start += partoff;
	start /= raw_page_size;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;

	for (i = 0; size; i++) {
		memcpy(databuf, data + raw_page_size * i, raw_page_size);
		err = flashdrv_writeraw(dma, start + i, databuf, raw_page_size);

		if (err) {
			LOG_ERROR("raw write error %d", err);
			break;
		}
		size -= raw_page_size;
	}
	writesz -= size;

	return writesz;
}


static int flashsrv_devWriteMeta(flash_i_devctl_t *idevctl, char* data)
{
	flashdrv_dma_t *dma;
	int i, err;
	char *databuf;
	size_t partoff;
	size_t size = idevctl->write.size;
	size_t writesz = idevctl->write.size;

	TRACE("META write off: %d, size: %d, ptr: %p", idevctl->write.address, size, data);

	if (size % flashsrv_common.info.writesz)
		return -EINVAL;

	if (idevctl->write.address % flashsrv_common.info.writesz)
		return -EINVAL;

	if ((err = flashsrv_partoff(idevctl->write.oid.id, 0, idevctl->write.address, idevctl->write.size, &partoff, NULL)) < 0)
		return err;

	idevctl->write.address += partoff;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;

	memcpy(databuf, data, flashsrv_common.info.writesz);
	for (i = 0; size; i++) {
		TRACE("  paddr: %u, eb: %u", idevctl->write.address / flashsrv_common.info.writesz, idevctl->write.address / flashsrv_common.info.erasesz);
		err = flashdrv_write(dma, idevctl->write.address / flashsrv_common.info.writesz + i, NULL, databuf);

		if (err) {
			LOG_ERROR("write error %d", err);
			break;
		}
		size -= flashsrv_common.info.writesz;
	}

	writesz -= size;

	return writesz;
}


static int flashsrv_devReadRaw(flash_i_devctl_t *idevctl, char *data)
{
	flashdrv_dma_t *dma;
	char *databuf;
	size_t rp, partoff, totalBytes = 0;
	const unsigned int raw_page_size = flashsrv_common.info.writesz + flashsrv_common.info.metasz;
	size_t size = idevctl->readraw.size;
	size_t offset = idevctl->readraw.address;
	int err = EOK;

	if ((size % raw_page_size) || (offset % raw_page_size))
		return -EINVAL;

	if ((err = flashsrv_partoff(idevctl->readraw.oid.id, 1, offset, size, &partoff, NULL)) < 0) /* raw = 1 */
		return err;

	offset += partoff;

	dma = flashsrv_common.dma;
	databuf = flashsrv_common.databuf;
	rp = offset / raw_page_size;

	while (size) {
		err = flashdrv_readraw(dma, rp, databuf, raw_page_size);
		memcpy(data, databuf, raw_page_size);

		if (err < 0) {
			LOG_ERROR("error in readraw(): %d", err);
			err = -EIO;
			break;
		}

		size -= raw_page_size;
		totalBytes += raw_page_size;
		data += raw_page_size;
		rp++;
	}

	return totalBytes;
}


static int flashsrv_devIsbad(const flash_i_devctl_t *idevctl)
{
	size_t address = idevctl->badblock.address;
	size_t partoff;
	int err;

	TRACE("DevIsbad: off:%zu", address);

	if (address % flashsrv_common.info.erasesz)
		return -EINVAL;

	if ((err = flashsrv_partoff(idevctl->badblock.oid.id, 0, address, flashsrv_common.info.erasesz, &partoff, NULL)) < 0)
		return err;

	address += partoff;

	return flashdrv_isbad(flashsrv_common.dma, address / flashsrv_common.info.writesz);
}


static int flashsrv_devMarkbad(const flash_i_devctl_t *idevctl)
{
	size_t address = idevctl->badblock.address;
	size_t partoff;
	int err;

	TRACE("DevMarkbad: off:%zu", address);

	if (address % flashsrv_common.info.erasesz)
		return -EINVAL;

	if ((err = flashsrv_partoff(idevctl->badblock.oid.id, 0, address, flashsrv_common.info.erasesz, &partoff, NULL)) < 0)
		return err;

	address += partoff;

	return flashdrv_markbad(flashsrv_common.dma, address / flashsrv_common.info.writesz);
}


static void flashsrv_devCtrl(msg_t *msg)
{
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	switch (idevctl->type) {
		case flashsrv_devctl_info:
			odevctl->err = 0;
			memcpy(&odevctl->info, &flashsrv_common.info, sizeof(flashsrv_info_t));
			break;

		case flashsrv_devctl_erase:
			odevctl->err = flashsrv_devErase(idevctl);
			break;

		case flashsrv_devctl_writeraw:
			odevctl->err = flashsrv_devWriteRaw(idevctl, msg->i.data);
			break;

		case flashsrv_devctl_writemeta:
			odevctl->err = flashsrv_devWriteMeta(idevctl, msg->i.data);
			break;

		case flashsrv_devctl_readraw:
			odevctl->err = flashsrv_devReadRaw(idevctl, msg->o.data);
			break;

		case flashsrv_devctl_isbad:
			odevctl->err = flashsrv_devIsbad(idevctl);
			break;

		case flashsrv_devctl_markbad:
			odevctl->err = flashsrv_devMarkbad(idevctl);
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
			return p->size * flashsrv_common.info.erasesz;

		case atDev:
			return p->start * flashsrv_common.info.erasesz;

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
			TRACE("DEV mtgetAttr - id: %llu", msg.i.attr.oid.id);
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
	TRACE("partition allocated, start: %u, size:%u,  id %d", start, size, idtree_id(&p->node));
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
	const flashdrv_info_t *drvinfo;
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
	drvinfo = flashdrv_info();
	printf("imx6ull-flash: %s\n", drvinfo->name);
	flashsrv_common.info.size = drvinfo->size;
	flashsrv_common.info.writesz = drvinfo->writesz;
	flashsrv_common.info.metasz = drvinfo->metasz;
	flashsrv_common.info.erasesz = drvinfo->erasesz;

	/* create root partition covering full interface */
	flashsrv_partition(0, flashsrv_common.info.size / flashsrv_common.info.erasesz);

	flashsrv_common.dma = flashdrv_dmanew();
	flashsrv_common.databuf = mmap(NULL, 2 * flashsrv_common.info.writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);
	flashsrv_common.metabuf = mmap(NULL, flashsrv_common.info.writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);

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

		if (oid.id == 0)
			sprintf(path, "flash0"); /* root device */
		else
			sprintf(path, "flash0p%u", (unsigned int)oid.id);

		create_dev(&oid, path);
	}

	if (rootfs == NULL)
		daemonize();
	else
		portRegister(rootfs->port, "/", &rootoid);

	printf("imx6ull-flash: initialized\n");
	flashsrv_devThread((void *)port);
}
