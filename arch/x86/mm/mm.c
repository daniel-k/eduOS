/*
 * Copyright (c) 2015, Daniel Krebs, RWTH Aachen University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the University nor the names of its contributors
 *      may be used to endorse or promote products derived from this
 *      software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <eduos/stddef.h>
#include <asm/mm.h>
#include <asm/page.h>
#include <eduos/vma.h>
#include <eduos/errno.h>

int kmmap_identity(size_t phy_addr, uint32_t flags)
{
	size_t page = phy_addr & PAGE_MASK;

	return kmap_page(page, page, flags);
}


int kmap_page(size_t phy_page, size_t virt_page, uint32_t flags)
{
	if( (phy_page  & ~PAGE_MASK) ||
	    (virt_page & ~PAGE_MASK) )
	{
		/* Arguments are not on page boundaries */
		return -EINVAL;
	}

	uint32_t paging_flags = 0;

	/* translate (vma) flags to paging flags*/
	if( flags & VMA_USER )
		paging_flags |= PG_USER;
	if( ! (flags & VMA_CACHEABLE) )
		paging_flags |= PG_PCD;
	if( (flags & (VMA_READ | VMA_WRITE)) && !(flags & VMA_NO_ACCESS) )
		paging_flags |= PG_RW;
#ifdef CONFIG_X86_64
	if( ! (flags & VMA_CACHEABLE) )
		paging_flags |= PG_XD;
#endif


	int ret;

	/* always use PG_GLOBAL because we only map for kernel space */
	if( (ret = page_map(virt_page, phy_page, 1, PG_GLOBAL | paging_flags)) )
		return ret;

	if( (ret = vma_add(virt_page, virt_page + PAGE_SIZE, flags)) )
		return ret;

	return 0;
}
