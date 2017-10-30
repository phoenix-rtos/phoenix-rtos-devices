#ifndef VYBRID_TAMPER_H
#define VYBRID_TAMPER_H

#define TAMPER_NUM 2

//masks for tamper_setCb
#define TAMPER_ID_1 0x1
#define TAMPER_ID_2 0x2


typedef void (*tamperCb_t)(void *);

int tamper_setCb(tamperCb_t cb, void *arg, unsigned tamper);

int tamper_resetCb(tamperCb_t cb, void *arg, unsigned tamper);

extern void tamper_init(void);

#endif
