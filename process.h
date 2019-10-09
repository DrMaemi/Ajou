/**********************************************************************
 * Copyright (c) 2019
 *  Sang-Hoon Kim <sanghoonkim@ajou.ac.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTIABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 **********************************************************************/

#ifndef __PROCESS_H__
#define __PROCESS_H__

struct list_head;

enum process_status {
	PROCESS_READY,
	PROCESS_RUNNING,
	PROCESS_WAIT,
};

struct process {
	unsigned int pid;		/* Process ID */

	enum process_status status;

	unsigned int age;		/* # of ticks since the process was forked */
	unsigned int lifespan;	/* The lifespan of the process. The process will
							   be exited with age == lifespan */

	unsigned int prio;		/* Priority. 0 by default. The larger, the more
							   important process it is */
	unsigned int prio_orig; /* The original priority of the process. Use to
							   implement PIP */

	struct list_head list;	/* list head for listing processes */

	/* DO NOT ACCESS FOLLOWING VARIABLES */
	unsigned int __starts_at;	/* When to fork the process */

	struct list_head __resources_to_acquire;
								/* Schedule to acquire resources */

	struct list_head __resources_holding;
								/* Resources that the process is currently holding */

/*====================================================================*/
/*     ******   PUT YOUR WORK BETWEEN THE BORDER LINES   ******       */


/*     ******   PUT YOUR WORK BETWEEN THE BORDER LINES   ******       */
/*====================================================================*/
};

/**
 * Support function to dump the process and resource status
 */
void dump_status(void);

#endif
