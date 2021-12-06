#include <sys/msg.h>
#include <sys/minmax.h>
#include <sys/list.h>
#include <sys/rb.h>
#include <sys/threads.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/reboot.h>
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

/* clang-format off */
#define LOG(str_, ...) do { printf("imx6ull-flash: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG(__FILE__ ":%d error: " str_, __LINE__, ##__VA_ARGS__)
#define TRACE(str_, ...) do { if (0) LOG(__FILE__ ":%d trace: " str_, __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */

#define FLASH_PART_ID 0

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
	size_t start;     /* in erase blocks */
	size_t size;      /* in erase blocks */
	const char *name; /* optional: partition name -> we will create symlink if non-null */
} flashsrv_partition_t;


struct {
	char poolStacks[4][4 * 4096] __attribute__((aligned(8)));

	rbtree_t filesystems;
	idtree_t partitions;
	flashsrv_request_t *queue;
	handle_t lock, cond;

	int rootfs_partid;
	const char *rootfs_fsname;

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


static flashsrv_filesystem_t *flashsrv_mountFs(flashsrv_partition_t *partition, unsigned mode, const char *name)
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


static int flashsrv_fileAttr(int type, id_t id, long long *attr)
{
	flashsrv_partition_t *p = NULL;

	mutexLock(flashsrv_common.lock);
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, id));
	mutexUnlock(flashsrv_common.lock);

	if (p == NULL)
		return -1;

	switch (type) {
		case atSize:
			*attr = (off_t)p->size * flashsrv_common.info.erasesz;
			break;

		case atDev:
			*attr = (off_t)p->start * flashsrv_common.info.erasesz;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
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
			msg.o.attr.err = flashsrv_fileAttr(msg.i.attr.type, msg.i.attr.oid.id, &msg.o.attr.val);
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


static int flashsrv_addPartition(size_t start, size_t size, const char *name)
{
	flashsrv_partition_t *p;
	int ret;

	p = malloc(sizeof(*p));

	if (p == NULL)
		return -1;

	p->start = start;
	p->size = size;
	p->name = (name != NULL) ? strdup(name) : NULL;

	mutexLock(flashsrv_common.lock);
	ret = idtree_alloc(&flashsrv_common.partitions, &p->node);
	mutexUnlock(flashsrv_common.lock);

	TRACE("partition allocated, start: %zu, size:%zu, name: %s, id:%d", start, size, name, ret);
	return ret;
}


static void daemonize(void)
{
	/* TODO: required when we are not root */
}


static int parse_opts(int argc, char **argv)
{
	int c, rootfs_first = -1, rootfs_second = -1;
	char *p, *part_name;
	size_t part_start, part_size;

	while ((c = getopt(argc, argv, "r:p:")) != -1) {
		switch (c) {
			case 'r': /* fs_name:rootfs_part_id[:secondary_rootfs_part_id] */

				flashsrv_common.rootfs_fsname = optarg;
				p = strchr(optarg, ':');
				if (!p) {
					LOG_ERROR("missing rootfs filesystem name");
					return -1;
				}

				*p++ = '\0';

				errno = 0;
				rootfs_first = strtol(p, &p, 10);
				if (*p++ == ':') {
					rootfs_second = strtol(p, &p, 10);
				}

				break;

			case 'p': /* start:size[:name] */
				/* TODO: switch to positional arguments to highlight partition order matter */

				errno = 0;
				part_start = strtol(optarg, &p, 10);
				if (*p++ != ':') {
					LOG_ERROR("missing partition size");
					return -1;
				}

				part_size = strtol(p, &p, 10);
				if (errno == ERANGE) {
					LOG_ERROR("partition parameters out of range");
					return -1;
				}

				part_name = (*p == ':') ? (p + 1) : NULL;

				if (flashsrv_addPartition(part_start, part_size, part_name) < 0) {
					LOG_ERROR("failed to add partition %zu:%zu", part_start, part_size);
					return -1;
				}
				break;

			default:
				break;
		}
	}

	/* TODO: add partition table support here */

	if ((flashsrv_common.rootfs_fsname == NULL) || (rootfs_first <= 0)) {
		/* code path for psu/psd */
		LOG("missing/invalid rootfs definition, not mounting '/'");
		return 0;
	}

	flashsrv_common.rootfs_partid = rootfs_first;
	if (rootfs_second > 0) {
		/* imx6ull-specific reboot reason - check if we're booting from secondary boot image */
		uint32_t reason = 0;
		if (reboot_reason(&reason) < 0)
			LOG_ERROR("reboot_reason: failed");

		if (reason & (1u << 30)) {
			LOG("using secondary boot image");
			flashsrv_common.rootfs_partid = rootfs_second;
		}
	}


	return 0;
}


/* create symlink manually as at this point we might not have '/' yet, so resolve_path would fail */
static int create_devfs_symlink(const char *name, const char *target)
{
	oid_t dir;
	msg_t msg = { 0 };
	int len1, len2;
	int ret;

	if ((ret = lookup("devfs", NULL, &dir)) < 0)
		return ret;

	msg.type = mtCreate;

	memcpy(&msg.i.create.dir, &dir, sizeof(oid_t));
	msg.i.create.type = otSymlink;
	/* POSIX: symlink file permissions are undefined, use sane default */
	msg.i.create.mode = S_IFLNK | 0777;

	len1 = strlen(name);
	len2 = strlen(target);

	msg.i.size = len1 + len2 + 2;
	msg.i.data = malloc(msg.i.size);
	if (msg.i.data == NULL)
		return -ENOMEM;

	memset(msg.i.data, 0, msg.i.size);

	memcpy(msg.i.data, name, len1);
	memcpy(msg.i.data + len1 + 1, target, len2);

	ret = msgSend(dir.port, &msg);
	free(msg.i.data);

	return ret != EOK ? -EIO : msg.o.create.err;
}


int main(int argc, char **argv)
{
	unsigned int i;
	oid_t oid = { 0 };
	const flashdrv_info_t *drvinfo;
	rbnode_t *n;
	unsigned port;
	char path[32];
	flashsrv_partition_t *p;
	flashsrv_filesystem_t *rootfs;
	int err;


	condCreate(&flashsrv_common.cond);
	mutexCreate(&flashsrv_common.lock);
	lib_rbInit(&flashsrv_common.filesystems, flashsrv_fscmp, NULL);
	idtree_init(&flashsrv_common.partitions);

	flashsrv_common.queue = NULL;

	/* reserve ID 0 for "full flash" partition - for now with bogus size */
	flashsrv_addPartition(0, 1, NULL);
	if (parse_opts(argc, argv) < 0)
		return 1;

	portCreate(&port);

	TRACE("got port %d", port);

	flashdrv_init();
	drvinfo = flashdrv_info();
	LOG("%s", drvinfo->name);
	flashsrv_common.info.size = drvinfo->size;
	flashsrv_common.info.writesz = drvinfo->writesz;
	flashsrv_common.info.metasz = drvinfo->metasz;
	flashsrv_common.info.erasesz = drvinfo->erasesz;

	/* update "full flash" partition size */
	p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, FLASH_PART_ID));
	p->size = flashsrv_common.info.size / flashsrv_common.info.erasesz;

	flashsrv_common.dma = flashdrv_dmanew();
	flashsrv_common.databuf = mmap(NULL, 2 * flashsrv_common.info.writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);
	flashsrv_common.metabuf = mmap(NULL, flashsrv_common.info.writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);

	for (i = 0; i < sizeof(flashsrv_common.poolStacks) / sizeof(flashsrv_common.poolStacks[0]); ++i)
		beginthread(flashsrv_poolThread, 4, flashsrv_common.poolStacks[i], sizeof(flashsrv_common.poolStacks[i]), NULL);

	/* create devices for all partitions */
	for (n = lib_rbMinimum(flashsrv_common.partitions.root); n; n = lib_rbNext(n)) {
		p = lib_treeof(flashsrv_partition_t, node, n);
		oid.id = idtree_id(&p->node);
		oid.port = port;

		if (oid.id == FLASH_PART_ID)
			sprintf(path, "flash0"); /* root device */
		else
			sprintf(path, "flash0p%u", (unsigned int)oid.id);

		if (create_dev(&oid, path) < 0)
			LOG_ERROR("failed to create device file for partition id: %ju\n", oid.id); /* assume it's non-fatal */

		LOG("%-8s <%4zu, %4zu>: name: %s", path, p->start, p->start + p->size, p->name ? p->name : "(nil)");

		if (p->name) {
			if ((err = create_devfs_symlink(p->name, path)) < 0)
				LOG_ERROR("symlink creation failed: %s", strerror(err));
		}
	}

	/* mount / symlink rootfs partition */
	if (flashsrv_common.rootfs_partid > 0) {
		p = lib_treeof(flashsrv_partition_t, node, idtree_find(&flashsrv_common.partitions, flashsrv_common.rootfs_partid));
		if (p == NULL) {
			LOG_ERROR("rootfs partition %d not found", flashsrv_common.rootfs_partid);
			return 1;
		}

		rootfs = flashsrv_mountFs(p, 0, flashsrv_common.rootfs_fsname);
		if (rootfs == NULL) {
			LOG_ERROR("failed to mount root partition");
			return 1;
		}

		oid.port = rootfs->port;
		oid.id = rootfs->root;
		portRegister(rootfs->port, "/", &oid);

		sprintf(path, "flash0p%u", (unsigned int)idtree_id(&p->node));
		LOG("mounting %s as a rootfs (%s)", path, flashsrv_common.rootfs_fsname);
		if ((err = create_devfs_symlink("root", path)) < 0)
			LOG_ERROR("root symlink creation failed: %s", strerror(err));
	}
	else {
		daemonize();
	}

	LOG("initialized");
	flashsrv_devThread((void *)port);
}
