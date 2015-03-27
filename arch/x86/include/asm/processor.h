/*
 * Copyright (c) 2010, Stefan Lankes, RWTH Aachen University
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
 */

/** 
 * @author Stefan Lankes
 * @file arch/x86/include/asm/processor.h
 * @brief CPU-specific functions
 *
 * This file contains structures and functions related to CPU-specific assembler commands.
 */

#ifndef __ARCH_PROCESSOR_H__
#define __ARCH_PROCESSOR_H__

#include <eduos/stddef.h>
#include <asm/gdt.h>
#include <asm/apic.h>
#include <asm/acpi.h>
#include <asm/irqflags.h>
#ifdef CONFIG_PCI
#include <asm/pci.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// feature list 1
#define CPU_FEATURE_FPU			(1 << 0)
#define CPU_FEATUE_PSE			(1 << 3)
#define CPU_FEATURE_MSR			(1 << 5)
#define CPU_FEATURE_PAE			(1 << 6)
#define CPU_FEATURE_APIC		(1 << 9)
#define CPU_FEATURE_SEP			(1 << 11)
#define CPU_FEATURE_PGE			(1 << 13)
#define CPU_FEATURE_PAT			(1 << 16)
#define CPU_FEATURE_PSE36		(1 << 17)
#define CPU_FEATURE_MMX			(1 << 23)
#define CPU_FEATURE_FXSR		(1 << 24)
#define CPU_FEATURE_SSE			(1 << 25)
#define CPU_FEATURE_SSE2		(1 << 26)

// feature list 2
#define CPU_FEATURE_X2APIC		(1 << 21)
#define CPU_FEATURE_AVX			(1 << 28)
#define CPU_FEATURE_HYPERVISOR	(1 << 31)

// CPUID.80000001H:EDX feature list
#define CPU_FEATURE_SYSCALL		(1 << 11)
#define CPU_FEATURE_NX			(1 << 20)
#define CPU_FEATURE_1GBHP		(1 << 26)
#define CPU_FEATURE_LM			(1 << 29)

// x86 control registers

/// Protected Mode Enable
#define CR0_PE					(1 << 0)
/// Monitor coprocessor
#define CR0_MP					(1 << 1)
/// Enable FPU emulation
#define CR0_EM					(1 << 2)
/// Task switched
#define CR0_TS					(1 << 3)
/// Extension type of coprocessor
#define CR0_ET					(1 << 4)
/// Enable FPU error reporting
#define CR0_NE					(1 << 5)
/// Enable write protected pages
#define CR0_WP					(1 << 16)
/// Enable alignment checks
#define CR0_AM					(1 << 18)
/// Globally enables/disable write-back caching
#define CR0_NW					(1 << 29)
/// Globally disable memory caching
#define CR0_CD					(1 << 30)
/// Enable paging
#define CR0_PG					(1 << 31)

/// Virtual 8086 Mode Extensions
#define CR4_VME					(1 << 0)
/// Protected-mode Virtual Interrupts
#define CR4_PVI					(1 << 1)
/// Disable Time Stamp Counter register (rdtsc instruction)
#define CR4_TSD					(1 << 2)
/// Enable debug extensions
#define CR4_DE					(1 << 3)
///  Enable hugepage support
#define CR4_PSE					(1 << 4)
/// Enable physical address extension
#define CR4_PAE					(1 << 5)
/// Enable machine check exceptions
#define CR4_MCE					(1 << 6)
/// Enable global pages
#define CR4_PGE					(1 << 7)
/// Enable Performance-Monitoring Counter
#define CR4_PCE					(1 << 8)
/// Enable Operating system support for FXSAVE and FXRSTOR instructions
#define CR4_OSFXSR				(1 << 9)
/// Enable Operating System Support for Unmasked SIMD Floating-Point Exceptions
#define CR4_OSXMMEXCPT			(1 << 10)
/// Enable Virtual Machine Extensions, see Intel VT-x
#define CR4_VMXE				(1 << 13)
/// Enable Safer Mode Extensions, see Trusted Execution Technology (TXT)
#define CR4_SMXE				(1 << 14)
/// Enables the instructions RDFSBASE, RDGSBASE, WRFSBASE, and WRGSBASE
#define CR4_FSGSBASE			(1 << 16)
/// Enables process-context identifiers
#define CR4_PCIDE				(1 << 17)
/// Enable XSAVE and Processor Extended States
#define CR4_OSXSAVE				(1 << 18)
/// Enable Supervisor Mode Execution Protection
#define CR4_SMEP				(1 << 20)
/// Enable Supervisor Mode Access Protection
#define CR4_SMAP				(1 << 21)

// x86-64 specific MSRs

/// APIC register
#define MSR_APIC_BASE			0x0000001B
/// extended feature register
#define MSR_EFER				0xc0000080
/// legacy mode SYSCALL target
#define MSR_STAR				0xc0000081
/// long mode SYSCALL target
#define MSR_LSTAR				0xc0000082
/// compat mode SYSCALL target
#define MSR_CSTAR				0xc0000083
/// EFLAGS mask for syscall
#define MSR_SYSCALL_MASK		0xc0000084
/// 64bit FS base
#define MSR_FS_BASE				0xc0000100
/// 64bit GS base
#define MSR_GS_BASE				0xc0000101
/// SwapGS GS shadow
#define MSR_KERNEL_GS_BASE		0xc0000102

// MSR EFER bits
#define EFER_SCE				(1 << 0)
#define EFER_LME				(1 << 8)
#define EFER_LMA				(1 << 10)
#define EFER_NXE				(1 << 11)
#define EFER_SVME				(1 << 12)
#define EFER_LMSLE				(1 << 13)
#define EFER_FFXSR				(1 << 14)
#define EFER_TCE				(1 << 15)

typedef struct {
	uint32_t feature1, feature2, feature3;
	uint32_t addr_width;
} cpu_info_t;

extern cpu_info_t cpu_info;

// determine the cpu features
int cpu_detection(void);

inline static uint32_t has_fpu(void) {
	return (cpu_info.feature1 & CPU_FEATURE_FPU);
}

inline static uint32_t has_msr(void) {
	return (cpu_info.feature1 & CPU_FEATURE_MSR);
}

inline static uint32_t has_apic(void) {
	return (cpu_info.feature1 & CPU_FEATURE_APIC);
}

inline static uint32_t has_fxsr(void) {
	return (cpu_info.feature1 & CPU_FEATURE_FXSR);
}

inline static uint32_t has_sse(void) {
	return (cpu_info.feature1 & CPU_FEATURE_SSE);
}

inline static uint32_t has_sse2(void) {
	return (cpu_info.feature1 & CPU_FEATURE_SSE2);
}

inline static uint32_t has_pge(void)
{
	return (cpu_info.feature1 & CPU_FEATURE_PGE);
}

inline static uint32_t has_sep(void) {
	return (cpu_info.feature1 & CPU_FEATURE_SEP);
}

inline static uint32_t has_x2apic(void) {
	return (cpu_info.feature2 & CPU_FEATURE_X2APIC);
}

inline static uint32_t has_avx(void) {
	return (cpu_info.feature2 & CPU_FEATURE_AVX);
}

inline static uint32_t on_hypervisor(void) {
	return (cpu_info.feature2 & CPU_FEATURE_HYPERVISOR);
}

inline static uint32_t has_nx(void)
{
	return (cpu_info.feature3 & CPU_FEATURE_NX);
}

/** @brief Read out time stamp counter
 *
 * The rdtsc asm command puts a 64 bit time stamp value
 * into EDX:EAX.
 *
 * @return The 64 bit time stamp value
 */
inline static uint64_t rdtsc(void)
{
#ifdef CONFIG_X86_32
	uint64_t x;
	asm volatile ("rdtsc" : "=A" (x));
	return x;
#elif defined(CONFIG_X86_64)
	uint64_t lo, hi;
	asm volatile ("rdtsc" : "=a"(lo), "=d"(hi) );
	return (hi << 32 | lo);
#endif
}

/** @brief Read MSR
 *
 * The asm instruction rdmsr which stands for "Read from model specific register"
 * is used here.
 *
 * @param msr The parameter which rdmsr assumes in ECX
 * @return The value rdmsr put into EDX:EAX
 */
inline static uint64_t rdmsr(uint32_t msr) {
	uint32_t low, high;

	asm volatile ("rdmsr" : "=a" (low), "=d" (high) : "c" (msr));

	return ((uint64_t)high << 32) | low;
}

/** @brief Write a value to a  Machine-Specific Registers (MSR)
 *
 * The asm instruction wrmsr which stands for "Write to model specific register"
 * is used here.
 *
 * @param msr The MSR identifier
 * @param value Value, which will be store in the MSR
 */
inline static void wrmsr(uint32_t msr, uint64_t value)
{
	uint32_t low = value & 0xFFFFFFFF;
	uint32_t high = value >> 32;

	asm volatile("wrmsr" :: "a"(low), "c"(msr), "d"(high));
}

/** @brief Read cr0 register
 * @return cr0's value
 */
static inline size_t read_cr0(void) {
	size_t val;
	asm volatile("mov %%cr0, %0" : "=r"(val));
	return val;
}

/** @brief Write a value into cr0 register
 * @param val The value you want to write into cr0
 */
static inline void write_cr0(size_t val) {
	asm volatile("mov %0, %%cr0" : : "r"(val));
}

/** @brief Read cr2 register
 * @return cr2's value
 */
static inline size_t read_cr2(void) {
	size_t val;
	asm volatile("mov %%cr2, %0" : "=r"(val));
	return val;
}

/** @brief Write a value into cr2 register
 * @param val The value you want to write into cr2
 */
static inline void write_cr2(size_t val) {
	asm volatile("mov %0, %%cr2" : : "r"(val));
}

/** @brief Read cr3 register
 * @return cr3's value
 */
static inline size_t read_cr3(void) {
        size_t val;
        asm volatile("mov %%cr3, %0" : "=r"(val));
        return val;
}

/** @brief Write a value into cr3 register
 * @param val The value you want to write into cr3
 */
static inline void write_cr3(size_t val) {
	asm volatile("mov %0, %%cr3" : : "r"(val));
}

/** @brief Read cr4 register
 * @return cr4's value
 */
static inline size_t read_cr4(void) {
	size_t val;
	asm volatile("mov %%cr4, %0" : "=r"(val));
	return val;
}

/** @brief Write a value into cr4 register
 * @param val The value you want to write into cr4
 */
static inline void write_cr4(size_t val) {
	asm volatile("mov %0, %%cr4" : : "r"(val));
}

/** @brief Flush cache
 *
 * The wbinvd asm instruction which stands for "Write back and invalidate"
 * is used here
 */
inline static void flush_cache(void) {
	asm volatile ("wbinvd" ::: "memory");
}

/** @brief Invalidate cache
 *
 * The invd asm instruction which invalidates cache without writing back
 * is used here
 */
inline static void invalidate_cache(void) {
	asm volatile ("invd" ::: "memory");
}

/** @brief Flush Translation Lookaside Buffer
 *
 * Just reads cr3 and writes the same value back into it.
 */
static inline void flush_tlb(void)
{
	size_t val = read_cr3();

	if (val)
		write_cr3(val);
}

/** @brief Flush a specific page entry in TLB
 * @param addr The (virtual) address of the page to flush
 */
static inline void tlb_flush_one_page(size_t addr)
{
	asm volatile("invlpg (%0)" : : "r"(addr) : "memory");
}

/** @brief Invalidate cache
 *
 * The invd asm instruction which invalidates cache without writing back
 * is used here
 */
inline static void invalid_cache(void) {
	asm volatile ("invd");
}

/* Force strict CPU ordering */
typedef void (*func_memory_barrier)(void);

/// Force strict CPU ordering, serializes load and store operations.
extern func_memory_barrier mb;
/// Force strict CPU ordering, serializes load operations.
extern func_memory_barrier rmb;
/// Force strict CPU ordering, serializes store operations.
extern func_memory_barrier wmb;

/** @brief Read out CPU ID
 *
 * The cpuid asm-instruction does fill some information into registers and
 * this function fills those register values into the given uint32_t vars.\n
 *
 * @param code Input parameter for the cpuid instruction. Take a look into the intel manual.
 * @param a EAX value will be stores here
 * @param b EBX value will be stores here
 * @param c ECX value will be stores here
 * @param d EDX value will be stores here
 */
inline static void cpuid(uint32_t code, uint32_t* a, uint32_t* b, uint32_t* c, uint32_t* d) {
	asm volatile ("cpuid" : "=a"(*a), "=b"(*b), "=c"(*c), "=d"(*d) : "0"(code), "2"(*c));
}

/** @brief Read EFLAGS
 *
 * @return The EFLAGS value
 */
static inline uint32_t read_eflags(void)
{
	uint32_t result;
	asm volatile ("pushf; pop %0" : "=r"(result));
	return result;
}

/** @brief search the first most significant bit
 *
 * @param i source operand
 * @return 
 * - first bit, which is set in the source operand
 * - invalid value, if not bit ist set
 */
static inline size_t msb(size_t i)
{
	size_t ret;

	if (!i)
		return (sizeof(size_t)*8);
	asm volatile ("bsr %1, %0" : "=r"(ret) : "r"(i) : "cc");

	return ret;
}

/** @brief search the least significant bit
 *
 * @param i source operand
 * @return 
 * - first bit, which is set in the source operand
 * - invalid value, if not bit ist set
 */
static inline size_t lsb(size_t i)
{
	size_t ret;

	if (!i)
		return (sizeof(size_t)*8);
	asm volatile ("bsf %1, %0" : "=r"(ret) : "r"(i) : "cc");

	return ret;
}

/// A one-instruction-do-nothing
#define NOP1	asm  volatile ("nop")
/// A two-instruction-do-nothing
#define NOP2	asm  volatile ("nop;nop")
/// A four-instruction-do-nothing
#define NOP4	asm  volatile ("nop;nop;nop;nop")
/// A eight-instruction-do-nothing
#define NOP8	asm  volatile ("nop;nop;nop;nop;nop;nop;nop;nop")
/// The PAUSE instruction provides a hint to the processor that the code sequence is a spin-wait loop.
#define PAUSE	asm volatile ("pause")
/// The HALT instruction stops the processor until the next interrupt arrives 
#define HALT	asm volatile ("hlt")

/** @brief Init several subsystems
 *
 * This function calls the initialization procedures for:
 * - GDT
 * - APIC
 * - PCI [if configured]
 *
 * @return 0 in any case
 */
inline static int system_init(void)
{
	gdt_install();
	cpu_detection();
#ifdef CONFIG_PCI
	pci_init();
#endif

	return 0;
}

/** @brief Detect and read out CPU frequency
 *
 * @return The CPU frequency in MHz
 */
uint32_t detect_cpu_frequency(void);

/** @brief Read out CPU frequency if detected before
 *
 * If you did not issue the detect_cpu_frequency() function before,
 * this function will call it implicitly.
 *
 * @return The CPU frequency in MHz
 */
uint32_t get_cpu_frequency(void);

/** @brief Busywait an microseconds interval of time
 * @param usecs The time to wait in microseconds
 */
void udelay(uint32_t usecs);

/** @brief System calibration
 *
 * This procedure will detect the CPU frequency and calibrate the APIC timer.
 *
 * @return 0 in any case.
 */
inline static int system_calibration(void)
{
	apic_init();
	acpi_init();
	irq_enable();
	detect_cpu_frequency();
	apic_calibration();

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif
