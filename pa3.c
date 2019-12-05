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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include "types.h"
#include "list_head.h"
#include "vm.h"

/**
 * Ready queue of the system
 */
extern struct list_head processes;

/**
 * The current process
 */
extern struct process *current;

/**
 * alloc_page()
 *
 * DESCRIPTION
 *   Allocate a page from the system. This function is implemented in vm.c
 *   and use to get a page frame from the system.
 *
 * RETURN
 *   PFN of the newly allocated page frame.
 */
extern unsigned int alloc_page(void);



/**
 * TODO translate()
 *
 * DESCRIPTION
 *   Translate @vpn of the @current to @pfn. To this end, walk through the
 *   page table of @current and find the corresponding PTE of @vpn.
 *   If such an entry exists and OK to access the pfn in the PTE, fill @pfn
 *   with the pfn of the PTE and return true.
 *   Otherwise, return false.
 *   Note that you should not modify any part of the page table in this function.
 *
 * RETURN
 *   @true on successful translation
 *   @false on unable to translate. This includes the case when @rw is for write
 *   and the @writable of the pte is false.
 */
bool translate(enum memory_access_type rw, unsigned int vpn, unsigned int *pfn)
{
	/*** DO NOT MODIFY THE PAGE TABLE IN THIS FUNCTION ***/
	if (!current->pagetable.outer_ptes[vpn >> PTES_PER_PAGE_SHIFT])
		return false;
	struct pte *pte = &(current->pagetable.outer_ptes[vpn >> PTES_PER_PAGE_SHIFT]->ptes[vpn % NR_PTES_PER_PAGE]);
	if (!pte->valid)
		return false;
	if (rw == WRITE && !pte->writable)
		return false;
	*pfn = current->pagetable.outer_ptes[vpn >> PTES_PER_PAGE_SHIFT]->ptes[vpn % NR_PTES_PER_PAGE].pfn;
	return true;
}


/**
 * TODO handle_page_fault()
 *
 * DESCRIPTION
 *   Handle the page fault for accessing @vpn for @rw. This function is called
 *   by the framework when the translate() for @vpn fails. This implies;
 *   1. Corresponding pte_directory is not exist
 *   2. pte is not valid
 *   3. pte is not writable but @rw is for write
 *   You can assume that all pages are writable; this means, when a page fault
 *   happens with valid PTE without writable permission, it was set for the
 *   copy-on-write.
 *
 * RETURN
 *   @true on successful fault handling
 *   @false otherwise
 */
bool handle_page_fault(enum memory_access_type rw, unsigned int vpn)
{
	if (!current->pagetable.outer_ptes[vpn >> PTES_PER_PAGE_SHIFT]) {
		struct pte_directory *tmp_directory;
		tmp_directory = malloc(sizeof(struct pte_directory));
		struct pte tmp;
		tmp.valid = false;
		tmp.writable = false;
		tmp.pfn = 0;
		for (int i = 0; i < NR_PTES_PER_PAGE; i++) 
			tmp_directory->ptes[i] = tmp;
		tmp_directory->ptes[vpn % NR_PTES_PER_PAGE].valid = true;
		tmp_directory->ptes[vpn % NR_PTES_PER_PAGE].writable = true;
		tmp_directory->ptes[vpn % NR_PTES_PER_PAGE].pfn = alloc_page(); 
		current->pagetable.outer_ptes[vpn >> PTES_PER_PAGE_SHIFT] = tmp_directory;
		return true;
	}
	struct pte *pte = &(current->pagetable.outer_ptes[vpn >> PTES_PER_PAGE_SHIFT]->ptes[vpn % NR_PTES_PER_PAGE]);

	if (!pte->valid){
		pte->valid = true;
		pte->writable = true;
		pte->pfn = alloc_page();
		return true;
	}

	if (rw == WRITE && !pte->writable) {
		pte->writable = true;
		pte->pfn = alloc_page();
		return true;
	}
	
	return false;
}


/**
 * TODO switch_process()
 *
 * DESCRIPTION
 *   If there is a process with @pid in @processes, switch to the process.
 *   The @current process at the moment should be put to the **TAIL** of the
 *   @processes list, and @current should be replaced to the requested process.
 *   Make sure that the next process is unlinked from the @processes.
 *   If there is no process with @pid in the @processes list, fork a process
 *   from the @current. This implies the forked child process should have
 *   the identical page table entry 'values' to its parent's (i.e., @current)
 *   page table. Also, should update the writable bit properly to implement
 *   the copy-on-write feature.
 */
void switch_process(unsigned int pid)
{
	if (current->pid == pid) return;
	struct process *next;
	struct process *p;
	if (!list_empty(&processes)) {
		list_for_each_entry(p, &processes, list) {
			if (p->pid == pid) {
				next = p;
				goto pick_next;
			}
		}
	}
	//There is no process whose pid is matched
	struct process *new_process;
	new_process = malloc(sizeof(struct process));
	new_process->pid = pid;
	for (int i = 0; i < NR_PTES_PER_PAGE; i++) {
		struct pte_directory *pd = current->pagetable.outer_ptes[i];
		if (!pd) continue;
		struct pte_directory *tmp_directory;
		tmp_directory = malloc(sizeof(struct pte_directory));
		for (int j = 0; j < NR_PTES_PER_PAGE; j++) {
			struct pte *pte = &pd->ptes[j];
			struct pte tmp;
			tmp.valid = pte->valid;
			tmp.writable = false;
			pte->writable = false;
			tmp.pfn = pte->pfn;
			tmp_directory->ptes[j] = tmp;
		}
		new_process->pagetable.outer_ptes[i] = tmp_directory;
	}
	list_add_tail(&current->list, &processes);
	current = new_process; return;

pick_next:
	list_add_tail(&current->list, &processes);
	list_del_init(&next->list);
	current = next;	
}

