// SPDX-License-Identifier: GPL-2.0-only
// Copyright 2023 Google LLC
// Author: Ard Biesheuvel <ardb@google.com>

#include <linux/types.h>
#include <linux/sizes.h>

#include <asm/memory.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>

#include "pi.h"

/*
 * DC-1 diagnostic only: page-table storage for an identity mapping of the
 * MediaTek watchdog page. Keeping it separate from INIT_IDMAP_DIR_SIZE avoids
 * consuming space reserved for the kernel's normal initial ID map.
 */
static u8 jagar_wdt_ptes[IDMAP_LEVELS - 1][PAGE_SIZE]
	__initdata __aligned(PAGE_SIZE);

/**
 * map_range - Map a contiguous range of physical pages into virtual memory
 *
 * @pte:		Address of physical pointer to array of pages to
 *			allocate page tables from
 * @start:		Virtual address of the start of the range
 * @end:		Virtual address of the end of the range (exclusive)
 * @pa:			Physical address of the start of the range
 * @prot:		Access permissions of the range
 * @level:		Translation level for the mapping
 * @tbl:		The level @level page table to create the mappings in
 * @may_use_cont:	Whether the use of the contiguous attribute is allowed
 * @va_offset:		Offset between a physical page and its current mapping
 * 			in the VA space
 */
void __init map_range(phys_addr_t *pte, u64 start, u64 end, phys_addr_t pa,
		      pgprot_t prot, int level, pte_t *tbl, bool may_use_cont,
		      u64 va_offset)
{
	u64 cmask = (level == 3) ? CONT_PTE_SIZE - 1 : U64_MAX;
	ptval_t protval = pgprot_val(prot) & ~PTE_TYPE_MASK;
	int lshift = (3 - level) * PTDESC_TABLE_SHIFT;
	u64 lmask = (PAGE_SIZE << lshift) - 1;

	start	&= PAGE_MASK;
	pa	&= PAGE_MASK;

	/* Advance tbl to the entry that covers start */
	tbl += (start >> (lshift + PAGE_SHIFT)) % PTRS_PER_PTE;

	/*
	 * Set the right block/page bits for this level unless we are
	 * clearing the mapping
	 */
	if (protval)
		protval |= (level == 2) ? PMD_TYPE_SECT : PTE_TYPE_PAGE;

	while (start < end) {
		u64 next = min((start | lmask) + 1, PAGE_ALIGN(end));

		if (level < 2 || (level == 2 && (start | next | pa) & lmask)) {
			/*
			 * This chunk needs a finer grained mapping. Create a
			 * table mapping if necessary and recurse.
			 */
			if (pte_none(*tbl)) {
				*tbl = __pte(__phys_to_pte_val(*pte) |
					     PMD_TYPE_TABLE | PMD_TABLE_UXN);
				*pte += PTRS_PER_PTE * sizeof(pte_t);
			}
			map_range(pte, start, next, pa, prot, level + 1,
				  (pte_t *)(__pte_to_phys(*tbl) + va_offset),
				  may_use_cont, va_offset);
		} else {
			/*
			 * Start a contiguous range if start and pa are
			 * suitably aligned
			 */
			if (((start | pa) & cmask) == 0 && may_use_cont)
				protval |= PTE_CONT;

			/*
			 * Clear the contiguous attribute if the remaining
			 * range does not cover a contiguous block
			 */
			if ((end & ~cmask) <= start)
				protval &= ~PTE_CONT;

			/* Put down a block or page mapping */
			*tbl = __pte(__phys_to_pte_val(pa) | protval);
		}
		pa += next - start;
		start = next;
		tbl++;
	}
}

asmlinkage phys_addr_t __init create_init_idmap(pgd_t *pg_dir, ptval_t clrmask)
{
	phys_addr_t ptep = (phys_addr_t)pg_dir + PAGE_SIZE; /* MMU is off */
	phys_addr_t wdt_ptep = (phys_addr_t)jagar_wdt_ptes;
	pgprot_t text_prot = PAGE_KERNEL_ROX;
	pgprot_t data_prot = PAGE_KERNEL;
	pgprot_t device_prot = __pgprot(PROT_DEVICE_nGnRnE);

	pgprot_val(text_prot) &= ~clrmask;
	pgprot_val(data_prot) &= ~clrmask;

	/* MMU is off; pointer casts to phys_addr_t are safe */
	map_range(&ptep, (u64)_stext, (u64)__initdata_begin,
		  (phys_addr_t)_stext, text_prot, IDMAP_ROOT_LEVEL,
		  (pte_t *)pg_dir, false, 0);
	map_range(&ptep, (u64)__initdata_begin, (u64)_end,
		  (phys_addr_t)__initdata_begin, data_prot, IDMAP_ROOT_LEVEL,
		  (pte_t *)pg_dir, false, 0);

	/*
	 * Permit one observable write after SCTLR_EL1.M is set. This mapping is
	 * device-nGnRnE and covers only the watchdog's 4 KiB register page.
	 */
	map_range(&wdt_ptep, 0x10007000, 0x10008000, 0x10007000,
		  device_prot, IDMAP_ROOT_LEVEL, (pte_t *)pg_dir, false, 0);

	return ptep;
}
