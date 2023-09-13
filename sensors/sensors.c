/*
 * Phoenix-RTOS
 *
 * Sensor Manager
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/list.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <sys/threads.h>

#include <sensors-spi.h>

#include "sensors.h"

#define CLIENT_SET_ID(id) (id + 1)
#define CLIENT_GET_ID(id) (id - 1)

#define NB_SENSOR_TYPES (sizeof(sensor_type_t) * 8)


typedef struct {
	sensors_ops_t ops;
	int refs;
	idnode_t node;
} sensor_client_t;


struct {
	rbtree_t drvs;    /* registered sensors drivers */
	idtree_t infos;   /* driver instances */
	idtree_t clients; /* set of clients */

	handle_t cLock;                          /* client's tree lock */
	handle_t locks[NB_SENSOR_TYPES];         /* locks for each sensor type */
	uint8_t evtNb[NB_SENSOR_TYPES];          /* number of events from all sensors */
	sensor_event_t *events[NB_SENSOR_TYPES]; /* each row defines set of events of the same type */

	uint8_t **devEvents; /* events assign to each device */

	oid_t oid;
} sensors_common;


/* Update data from sensors */

int sensors_publish(unsigned int devId, const sensor_event_t *event)
{
	int err = EOK;
	uint8_t id = __builtin_ffs(event->type);
	uint8_t evtId;

	/*__builtin_ffs returns one plus the index of the least significant, otherwise 0 */
	if (id == 0) {
		return -EINVAL;
	}

	--id;
	evtId = sensors_common.devEvents[devId][id];

	mutexLock(sensors_common.locks[id]);
	if (id >= NB_SENSOR_TYPES || sensors_common.events[id] == NULL || sensors_common.evtNb[id] < evtId) {
		err = -EINVAL;
	}
	memcpy(&sensors_common.events[id][evtId], event, sizeof(sensor_event_t));
	mutexUnlock(sensors_common.locks[id]);

	return err;
}


/* Handle messages */

static sensor_client_t *sensors_clientFind(id_t id)
{
	sensor_client_t *client;

	mutexLock(sensors_common.cLock);
	client = lib_treeof(sensor_client_t, node, idtree_find(&sensors_common.clients, id));
	if (client != NULL) {
		client->refs++;
	}
	mutexUnlock(sensors_common.cLock);

	return client;
}


static void sensors_clientPut(sensor_client_t *client)
{
	int refs;

	mutexLock(sensors_common.cLock);
	refs = --client->refs;
	if (refs <= 0) {
		idtree_remove(&sensors_common.clients, &client->node);
	}
	mutexUnlock(sensors_common.cLock);

	if (refs <= 0) {
		free(client);
	}
}


static int sensors_open(void)
{
	int res;
	sensor_client_t *client = calloc(1, sizeof(sensor_client_t));

	if (client == NULL) {
		return -ENOMEM;
	}

	client->refs = 1;
	mutexLock(sensors_common.cLock);
	res = idtree_alloc(&sensors_common.clients, &client->node);
	mutexUnlock(sensors_common.cLock);

	if (res < 0) {
		free(client);
	}
	else {
		res = CLIENT_SET_ID(res);
	}

	return res;
}


static int sensors_close(id_t id)
{
	int err = -EINVAL;
	sensor_client_t *client;

	client = sensors_clientFind(id);
	if (client != NULL) {
		sensors_clientPut(client);
		sensors_clientPut(client);
		err = EOK;
	}

	return err;
}


static ssize_t sensors_read(sensors_data_t *data, size_t sz, id_t clientID)
{
	uint8_t id;
	ssize_t res = -EINVAL;
	sensor_type_t types;
	size_t tempSz = 0, evtNb, chunkSz;
	sensor_client_t *client;
	sensor_event_t *events;

	if (data == NULL) {
		return res;
	}

	if (sz == 0) {
		return 0;
	}

	tempSz = sizeof(data->size);
	events = data->events;

	client = sensors_clientFind(clientID);
	if (client != NULL) {
		types = client->ops.types;
		/* Iterate only through available sensors for a client */
		for (id = __builtin_ffs(types); id != 0; types &= ~(1 << (id - 1)), id = __builtin_ffs(types)) {
			evtNb = sensors_common.evtNb[id - 1];
			chunkSz = evtNb * sizeof(sensor_event_t);
			if (tempSz + chunkSz > sz) {
				break;
			}

			mutexLock(sensors_common.locks[id - 1]);
			memcpy(events, sensors_common.events[id - 1], chunkSz);
			mutexUnlock(sensors_common.locks[id - 1]);

			events += evtNb;
			tempSz += chunkSz;
		}

		data->size = (tempSz - sizeof(data->size)) / sizeof(sensor_event_t);
		res = tempSz;

		sensors_clientPut(client);
	}

	return res;
}


/* Function return total number of events for selected types */
static unsigned int sensors_evtsCnt(sensor_type_t types)
{
	uint8_t id;
	unsigned int sz = 0;

	for (id = __builtin_ffs(types); id != 0; types &= ~(1 << (id - 1)), id = __builtin_ffs(types)) {
		sz += sensors_common.evtNb[id - 1];
	}

	return sz;
}


/* Function filters requested types with the available ones */
static sensor_type_t sensors_checkTypes(sensor_type_t types)
{
	uint8_t id;
	sensor_type_t avail = 0;

	for (id = __builtin_ffs(types); id != 0; types &= ~(1 << (id - 1)), id = __builtin_ffs(types)) {
		if (sensors_common.evtNb[id - 1] != 0) {
			avail |= (1 << (id - 1));
		}
	}

	return avail;
}


static void sensors_ioctl(msg_t *msg)
{
	id_t id;
	int i, err = EOK;
	unsigned long req;
	sensors_ops_t ops;
	sensor_type_t types = 0;
	sensor_client_t *client;
	void *outData = NULL;

	const void *inData = ioctl_unpack(msg, &req, &id);

	id = CLIENT_GET_ID(id);

	client = sensors_clientFind(id);
	if (client != NULL) {
		switch (req) {
			case SMIOC_SENSORSSET:
				ops.types = sensors_checkTypes(((sensors_ops_t *)inData)->types);
				ops.evtSz = sensors_evtsCnt(ops.types);

				client->ops.types = ops.types;
				client->ops.evtSz = ops.evtSz;

				outData = (void *)&ops;
				break;

			case SMIOC_SENSORSAVAIL:
				for (i = 0; i < NB_SENSOR_TYPES; ++i) {
					if (sensors_common.evtNb[i] != 0) {
						types |= (1 << i);
					}
				}
				outData = (void *)&types;
				break;

			default:
				break;
		}

		sensors_clientPut(client);
	}
	else {
		err = -EINVAL;
	}

	ioctl_setResponse(msg, req, err, outData);
}


static void sensors_msgThread(void)
{
	msg_t msg;
	msg_rid_t rid;

	while (1) {
		if (msgRecv(sensors_common.oid.port, &msg, &rid) < 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.io.err = sensors_open();
				break;

			case mtClose:
				msg.o.io.err = sensors_close(CLIENT_GET_ID(msg.i.openclose.oid.id));
				break;

			case mtRead:
				msg.o.io.err = sensors_read(msg.o.data, msg.o.size, CLIENT_GET_ID(msg.i.io.oid.id));
				break;

			case mtDevCtl:
				sensors_ioctl(&msg);
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(sensors_common.oid.port, &msg, rid);
	}
}


/* Sensor libraries management functions */

static int sensors_cmpDrvs(rbnode_t *n1, rbnode_t *n2)
{
	sensor_drv_t *drv1 = lib_treeof(sensor_drv_t, node, n1);
	sensor_drv_t *drv2 = lib_treeof(sensor_drv_t, node, n2);

	return strcmp(drv1->name, drv2->name);
}


static const sensor_drv_t *sensors_getDrv(const char *name)
{
	sensor_drv_t drv;

	strncpy(drv.name, name, sizeof(drv.name));
	drv.name[sizeof(drv.name) - 1] = '\0';

	return lib_treeof(sensor_drv_t, node, lib_rbFind(&sensors_common.drvs, &drv.node));
}


void sensors_register(const sensor_drv_t *drv)
{
	if (lib_rbInsert(&sensors_common.drvs, (rbnode_t *)&drv->node) != NULL) {
		fprintf(stderr, "sensors: cannot register driver: %s\n", drv->name);
	}
}


static void sensors_run(void)
{
	rbnode_t *node;
	sensor_info_t *info;
	const sensor_drv_t *drv;

	for (node = lib_rbMinimum(sensors_common.infos.root); node != NULL; node = lib_rbNext(node)) {
		info = lib_treeof(sensor_info_t, node, node);

		drv = sensors_getDrv(info->drv);
		if (drv == NULL) {
			fprintf(stderr, "sensors: cannot find driver for %s\n", info->drv);
			continue;
		}

		drv->start(info);
	}
}


/* Initialization functions */

static void sensors_help(const char *prog)
{
	printf("Usage: %s [options] or no args to automatically detect and initialize a NOR flash device\n", prog);
	printf("\t-s <name:args>          - initialize new sensor\n");
	printf("\t\tname:    name of the sensor driver\n");
	printf("\t\targs:    arguments passed to the sensor driver\n");
	printf("\t-h                      - print this help message\n");
}


static int sensors_initEvts(int devsz)
{
	int i, j, res;
	unsigned int id;
	rbnode_t *node;
	const sensor_info_t *info;

	if (devsz <= 0) {
		fprintf(stderr, "sensors: wrong device number\n");
		return -EINVAL;
	}

	/* Initialize device table */
	sensors_common.devEvents = malloc(devsz * sizeof(uint8_t *));
	if (sensors_common.devEvents == NULL) {
		fprintf(stderr, "sensors: cannot allocate memory\n");
		return -ENOMEM;
	}

	/* Each device can publish data of all sensor types. The following table contains information about device's events positions
	   in sensors_common.events tables */
	for (i = 0; i < devsz; ++i) {
		sensors_common.devEvents[i] = calloc(1, NB_SENSOR_TYPES);
		if (sensors_common.devEvents[i] == NULL) {
			fprintf(stderr, "sensors: cannot allocate memory\n");
			return -ENOMEM;
		}
	}

	/* Assign position of events for each device and calculate number of events of all types */
	for (node = lib_rbMinimum(sensors_common.infos.root), i = 0; node != NULL; node = lib_rbNext(node), ++i) {
		info = lib_treeof(sensor_info_t, node, node);

		for (j = 0; j < NB_SENSOR_TYPES; ++j) {
			id = 1 << j;
			if ((info->types & id) != 0) {
				sensors_common.devEvents[i][j] = sensors_common.evtNb[j]++;
			}
		}
	}

	/* Allocate memory for events tables */
	for (i = 0; i < NB_SENSOR_TYPES; ++i) {
		if (sensors_common.evtNb[i] != 0) {
			sensors_common.events[i] = malloc(sizeof(sensor_event_t) * sensors_common.evtNb[i]);
			if (sensors_common.events[i] == NULL) {
				fprintf(stderr, "sensors: cannot allocate memory for events\n");
				continue;
			}

			res = mutexCreate(&sensors_common.locks[i]);
			if (res < 0) {
				free(sensors_common.events[i]);
				fprintf(stderr, "sensors: cannot create lock\n");
			}
		}
	}

	return EOK;
}


static int sensors_drvInit(const char *name, const char *args)
{
	int res;
	sensor_info_t *info;
	const sensor_drv_t *drv;

	drv = sensors_getDrv(name);
	if (drv == NULL) {
		fprintf(stderr, "sensors: cannot find driver for %s\n", name);
		return -EINVAL;
	}

	info = malloc(sizeof(sensor_info_t));
	if (info == NULL) {
		fprintf(stderr, "sensors: cannot allocate memory\n");
		return -ENOMEM;
	}

	res = drv->alloc(info, args);
	if (res < 0) {
		free(info);
		fprintf(stderr, "sensors: cannot allocate sensor %s, err: %d\n", name, res);
		return res;
	}

	res = idtree_alloc(&sensors_common.infos, &info->node);
	if (res < 0) {
		free(info);
		fprintf(stderr, "sensors: cannot add sensor %s, to tree - err: %d\n", name, res);
		return res;
	}

	info->id = res;
	info->drv = drv->name;

	return res;
}


static int sensors_parseArgs(int argc, char **argv)
{
	int c, devsz = 0;
	char *args;
	const char *name;

	while ((c = getopt(argc, argv, "s:h")) != -1) {
		switch (c) {
			case 's': /* <sensor:args> */
				name = optarg;
				args = strchr(optarg, ':');
				if (args == NULL) {
					fprintf(stderr, "sensors: missing a sensors arguments: %s\n", optarg);
					return devsz;
				}

				*(args++) = '\0';
				if (sensors_drvInit(name, args) >= 0) {
					++devsz;
				}
				break;

			case 'h':
				sensors_help(argv[0]);
				return 0;

			default:
				return -EINVAL;
		}
	}

	return devsz;
}


static int sensors_createDev(void)
{
	oid_t root;
	int err;

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0) {
		usleep(10000);
	}

	sensors_common.oid.id = 0;
	err = portCreate(&sensors_common.oid.port);
	if (err < 0) {
		fprintf(stderr, "sensors: could not create a port\n");
		return err;
	}

	err = create_dev(&sensors_common.oid, "/dev/sensors");
	if (err) {
		fprintf(stderr, "sensors: could not create a device\n");
		return -EINVAL;
	}

	return EOK;
}


void sensors_cleanup(size_t devsz)
{
	int i;
	rbnode_t *node;
	sensor_info_t *info;
	sensor_client_t *client;

	/* Free device events tables */
	if (sensors_common.devEvents != NULL) {
		for (i = 0; i < devsz; ++i) {
			if (sensors_common.devEvents[i] != NULL) {
				free(sensors_common.devEvents[i]);
			}
		}
		free(sensors_common.devEvents);
	}

	/* Free sensor table and destroy mutexex */
	for (i = 0; i < NB_SENSOR_TYPES; ++i) {
		if (sensors_common.events[i] != NULL) {
			free(sensors_common.events[i]);
		}

		if (sensors_common.evtNb[i] != 0) {
			resourceDestroy(sensors_common.locks[i]);
		}
	}

	/* Free sensor information data */
	for (node = lib_rbMinimum(sensors_common.infos.root); node != NULL; node = lib_rbNext(node)) {
		info = lib_treeof(sensor_info_t, node, node);

		idtree_remove(&sensors_common.infos, &info->node);
		free(info);
	}

	/* Free clients */
	for (node = lib_rbMinimum(sensors_common.clients.root); node != NULL; node = lib_rbNext(node)) {
		client = lib_treeof(sensor_client_t, node, node);

		idtree_remove(&sensors_common.clients, &client->node);
		free(client);
	}

	resourceDestroy(sensors_common.cLock);
}


int main(int argc, char **argv)
{
	int devsz, res;

	res = mutexCreate(&sensors_common.cLock);
	if (res < 0) {
		sensors_cleanup(0);
		return EXIT_FAILURE;
	}

	res = sensorsspi_init();
	if (res < 0) {
		sensors_cleanup(0);
		return EXIT_FAILURE;
	}

	/* Initialize sensors connected to the board */
	devsz = sensors_parseArgs(argc, argv);
	res = sensors_initEvts(devsz);
	if (res < 0) {
		sensors_cleanup(devsz);
		return EXIT_FAILURE;
	}

	res = sensors_createDev();
	if (res < 0) {
		sensors_cleanup(devsz);
		return EXIT_FAILURE;
	}

	priority(THREAD_PRIORITY_MSGSRV);

	sensors_run();
	sensors_msgThread();

	return EXIT_SUCCESS;
}


void __attribute__((constructor(101))) sensors_init(void)
{
	/* Initialize data structures */
	lib_rbInit(&sensors_common.drvs, sensors_cmpDrvs, NULL);
	idtree_init(&sensors_common.infos);
	idtree_init(&sensors_common.clients);
}
