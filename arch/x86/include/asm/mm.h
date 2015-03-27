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

/**
 * @author Daniel Krebs
 * @file arch/x86/include/asm/mm.h
 * @brief Memory related functions
 *
 * This file contains platform independent memory functions
 */

#ifndef __ARCH_MM_H__
#define __ARCH_MM_H__

#include <eduos/stddef.h>
#include <eduos/vma.h>

/* @brief Map one page including phy_addr to the same virtual address space
 *
 * Short helper to access small memory portions in lower memory by the same,
 * i.e. physical, addresses.
 *
 * @param phy_addr  The physical address you want to access
 * @param flags     Flags see eduos/vma.h
 * @return
 */
int kmmap_identity(size_t phy_addr, uint32_t flags);

/* @brief Map one physical page to virtual page and add in vma entry
 *
 * Map one page from physical to virtual memory and keep track of mapping by
 * calling vma_add() internally. This also unifies flags because it converts
 * VMA_* flags to page flags (i.e. PG_*).
 *
 * @param phy_page      The physical page you want to access
 * @param virt_page     The physical page you want to access
 * @param flags         Flags see eduos/vma.h
 * @return
 */
int kmap_page(size_t phy_page, size_t virt_page, uint32_t flags);

#endif // __ARCH_MM_H__
