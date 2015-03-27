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
 * 2015-03-27:
 *  Some structs partially taken from https://github.com/RWTH-OS/smp.boot
 *  by Georg Wassen.
 *
 */

#ifndef __ARCH_ACPI_H__
#define __ARCH_ACPI_H__

#include <eduos/stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Places to search for ACPI tables, refer ACPIspec40a.pdf, p. 111 */

/* Extended BIOS Data Area */
#define EBDA_ADDRESS 0x9FC00
#define EBDA_LIMIT 0xA0000

/* Highest 2 kilobyte of lower memory */
#define BIOS_ROM_ADDRESS 0xE0000
#define BIOS_ROM_LIMIT 0x100000

// ----------------------------------------------------------------------------

/*
 * RSDP - Root System Description Pointer
 * (ACPIspec40a.pdf, p. 112)
 */
typedef struct {
	uint8_t signature[8];
	uint8_t checksum;
	uint8_t oemid[6];
	uint8_t revision;
	uint32_t rsdt_adr;
	uint32_t length;
	uint64_t xsdt_adr;
	uint8_t extended_checksum;
	uint8_t reserved[3];
} __attribute__((packed)) acpi_rsdp_t;


// ----------------------------------------------------------------------------

/*
* System Description Table Header
* (ACPIspec40a.pdf, p. 113)
*
* The following tables begin with this header.
*/
typedef struct {
	uint32_t signature; /* see: ACPIspec40a.pdf, Table 5-5 and 5-6, p. 114 */
	uint32_t length;
	uint8_t revision;
	uint8_t checksum;
	uint8_t oemid[6];
	uint8_t oem_table_id[8];
	uint32_t oem_revision;
	uint8_t creator_id[4];
	uint32_t creater_revision;
} __attribute__((packed)) acpi_sdt_header_t;

// ----------------------------------------------------------------------------

/*
 * RSDT - Root System Description Table
 * (ACPIspec40a.pdf, p. 116)
 */
typedef struct {
    acpi_sdt_header_t header;   /* signature: RSDT */
    uint32_t entry[];           /* number of entries is calculated from header.length */
} __attribute__((packed)) acpi_rsdt_t;

// ----------------------------------------------------------------------------

/*
* MADT - Multiple APIC Description Table
* (ACPIspec40a.pdf, p. 136)
*/
#define MADT_SIGNATURE ('A'|'P'<<8|'I'<<16|'C'<<24)

/* Every entry in MADT will use this header */
typedef struct {
	uint8_t type;
	uint8_t length;
} __attribute__((packed)) acpi_madt_entry_header_t;

typedef struct {
	acpi_sdt_header_t header;               /* signature: APIC */
	uint32_t lapic_addr;
	struct {
		uint32_t pcat_compat : 1;
		uint32_t reserved : 31;
	} flags;
	acpi_madt_entry_header_t apic_structs;  /* number of entries (they're of variable size...) must be derived from header.length */
} __attribute__((packed)) acpi_madt_t;

#define MADT_TYPE_LAPIC 0
#define MADT_TYPE_IOAPIC 1
#define MADT_TYPE_INTSRC 2
#define MADT_TYPE_LAPIC_NMI 4

/* Processor Local APIC entry */
typedef struct {
	acpi_madt_entry_header_t header;
	uint8_t processor_id;
	uint8_t apic_id;
	struct {
		uint32_t enabled : 1;
		uint32_t reserved : 31;
	} flags;
} __attribute__((packed)) acpi_madt_processor_lapic_entry_t;

/* I/O APIC entry */
typedef struct {
	acpi_madt_entry_header_t header;
	uint8_t io_apic_id;
	uint8_t reserved;
	uint32_t io_apic_adr;
	uint32_t global_irq_base;
} __attribute__((packed)) acpi_madt_io_apic_entry_t;

/* Interrupt Source Override */
typedef struct {
	acpi_madt_entry_header_t header;
	uint8_t bus;
	uint8_t source;
	uint32_t global_irq;
	struct {
			uint16_t polarity : 2;
			uint16_t trigger_mode : 2;
			uint16_t reserved : 12;
		} flags;
} __attribute__((packed)) acpi_madt_irq_source_override_entry_t;

// ----------------------------------------------------------------------------

void acpi_init();

#ifdef __cplusplus
}
#endif

#endif // __ARCH_ACPI_H__
