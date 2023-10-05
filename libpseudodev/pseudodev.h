/*
 * Phoenix-RTOS
 *
 * pseudo devices
 *
 * Copyright 2023 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * %LICENSE%
 */

#ifndef PSEUDO_H
#define PSEUDO_H


/* clang-format off */

enum { pseudo_idNull = 0, pseudo_idZero, pseudo_idFull, pseudo_idRandom };

/* clang-format on */


int pseudo_handleMsg(msg_t *msg, int id);


void pseudo_init(void);


#endif /* end of PSEUDO_H */
