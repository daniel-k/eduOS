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
 *  Some code taken or inspired from https://github.com/RWTH-OS/smp.boot
 *  by Georg Wassen
 *
 */

#include <eduos/stddef.h>
#include <eduos/string.h>
#include <asm/acpi.h>
#include <asm/page.h>
#include <eduos/stdio.h>
#include <eduos/processor.h>
#include <eduos/vma.h>
#include <asm/mm.h>
#include <eduos/errno.h>

acpi_rsdp_t* acpi_rsdp __attribute__ ((section (".data"))) = NULL;
acpi_rsdt_t* acpi_rsdt __attribute__ ((section (".data"))) = NULL;
acpi_madt_t* acpi_madt __attribute__ ((section (".data"))) = NULL;


/* Signature of RSDP */
union {
    char str[9];
    uint32_t u32[2];
} __attribute__((packed)) acpi_sig = { "RSD PTR " };


// ----------------------------------------------------------------------------

/* Calculate checksum of ACPI tables */
static int
acpi_checksum(void *hdr, size_t length)
{
    uint8_t sum = 0;
    uint8_t *p = (uint8_t*) hdr;

    unsigned i;
    for (i = 0; i < length; i++) {
        sum += p[i];
    }

    return sum;
}

// ----------------------------------------------------------------------------

/* Print common header of all ACPI tables (except RSDP) */
static void
print_acpi_header(acpi_sdt_header_t* hdr)
{
	if(!hdr)
		return;

	char* sig = (char*) &hdr->signature;

	kprintf("Table '%c%c%c%c':\n", sig[0], sig[1], sig[2], sig[3]);
	kprintf("  Length: %u\n", hdr->length);
	kprintf("  Revision: %u\n", hdr->revision);
	kprintf("  OEM id: \"%c%c%c%c%c%c\"\n",
			hdr->oemid[0],
			hdr->oemid[1],
			hdr->oemid[2],
			hdr->oemid[3],
			hdr->oemid[4],
			hdr->oemid[5]);
	kprintf("  OEM table id: \"%c%c%c%c%c%c%c%c\"\n",
			hdr->oem_table_id[0],
			hdr->oem_table_id[1],
			hdr->oem_table_id[2],
			hdr->oem_table_id[3],
			hdr->oem_table_id[4],
			hdr->oem_table_id[5],
			hdr->oem_table_id[6],
			hdr->oem_table_id[7]);
	kprintf("  OEM rev: %u\n", hdr->oem_revision);
	kprintf("  Creator id: \"%c%c%c%c\"\n",
			hdr->creator_id[0],
			hdr->creator_id[1],
			hdr->creator_id[2],
			hdr->creator_id[3]);
	kprintf("  Creator rev: %u\n", hdr->creater_revision);
}

// ----------------------------------------------------------------------------

static acpi_rsdp_t*
search_rdsp(size_t base, size_t limit)
{
	size_t ptr=PAGE_CEIL(base), vptr=0;
	acpi_rsdp_t* tmp;
	uint32_t i;

	while( ptr <= (limit - sizeof(acpi_rsdp_t)) ) {
		if (vptr) {
			// unmap page via mapping a zero page
			page_unmap(vptr, 1);
			vptr = 0;
		}

		if (BUILTIN_EXPECT(!page_map(ptr & PAGE_MASK, ptr & PAGE_MASK, 1, PG_GLOBAL | PG_RW | PG_PCD), 1))
			vptr = ptr & PAGE_MASK;
		else
			return NULL;

		for(i=0; (vptr) && (i<PAGE_SIZE-sizeof(acpi_rsdp_t)); i+=4, vptr+=4) {
			tmp = (acpi_rsdp_t*) vptr;

			uint32_t* sig = (uint32_t*)tmp->signature;

			if(sig[0] == acpi_sig.u32[0] && sig[1] == acpi_sig.u32[1])
			{
				if(acpi_checksum(tmp, 20) == 0)
				{
					vma_add(ptr & PAGE_MASK, (ptr & PAGE_MASK) + PAGE_SIZE, VMA_READ|VMA_WRITE);
					return tmp;
				}
			}
		}
		ptr += PAGE_SIZE;
	}

	if (vptr) {
		// unmap page via mapping a zero page
		page_unmap(vptr, 1);
	}
	return NULL;
}

// ----------------------------------------------------------------------------

static int
parse_madt(acpi_madt_t* madt)
{
	if(!madt)
		return -ENOENT;

	print_acpi_header((acpi_sdt_header_t*) madt);

	/* Local APIC Address */
	size_t lapic_addr = madt->lapic_addr;
	(void) lapic_addr;

	/* The first  entry in the list */
	uint8_t* ptr = (uint8_t*) &madt->apic_structs;

	acpi_madt_entry_header_t* entry;
	acpi_madt_processor_lapic_entry_t* processor_entry;
	acpi_madt_io_apic_entry_t* ioapic_entry;
	acpi_madt_irq_source_override_entry_t* irq_source_override;

	unsigned int i = 0;
	while( (__builtin_offsetof(acpi_madt_t, apic_structs) + i) < madt->header.length)
	{
		/* Get pointer to current entry */
		entry = (acpi_madt_entry_header_t*) (ptr + i);

		switch(entry->type)
		{
		case MADT_TYPE_LAPIC:
			/* Processor Local APIC */
			processor_entry = (acpi_madt_processor_lapic_entry_t*) entry;

			// TODO: Use these values instead of printing out
			kprintf("  Entry 'Processor Local APIC':\n");
			kprintf("    Processor ID: %u\n", processor_entry->processor_id);
			kprintf("    APIC ID: %u\n", processor_entry->apic_id);
			kprintf("    Enabled: %s\n", processor_entry->flags.enabled == 1 ? "yes" : "no");
			break;

		case MADT_TYPE_IOAPIC:
			/* I/O APIC */
			ioapic_entry = (acpi_madt_io_apic_entry_t*) entry;

			// TODO: Use these values instead of printing out
			kprintf("  Entry 'I/O APIC':\n");
			kprintf("    I/O APIC ID: %u\n", ioapic_entry->io_apic_id);
			kprintf("    I/O APIC Address: 0x%x\n", ioapic_entry->io_apic_adr);
			kprintf("    Global System Interrupt Base: %u\n", ioapic_entry->global_irq_base);
			break;

		case MADT_TYPE_INTSRC:
			/* Interrupt Source Override */
			irq_source_override = (acpi_madt_irq_source_override_entry_t*) entry;

			// TODO: Use these values instead of printing out
			kprintf("  Entry 'Interrupt Source Override':\n");
			kprintf("    Bus: %u\n", irq_source_override->bus);
			kprintf("    Source: %u\n", irq_source_override->source);
			kprintf("    Global System Interrupt: %u\n", irq_source_override->global_irq);
			kprintf("    Polarity: %u\n", irq_source_override->flags.polarity);
			kprintf("    Trigger Mode: %u\n", irq_source_override->flags.trigger_mode);
			break;

		default:
			kprintf("MADT entry of type %u not implemented yet\n", entry->type);
			break;
		}

		i += entry->length;
	}

	return 0;
}

// ----------------------------------------------------------------------------

/* @brief Parse tables referencend in RSDT
 *
 * @param rsdt  Pointer to RSDT table
 * @return      0 if all tables are successfully parsed, error code otherwise
 */
static int
parse_rsdt(acpi_rsdt_t* rsdt)
{
	if(!rsdt)
		return -ENOENT;

	/* Entries are 32 bit physical addresses that point to sub tables */
	uint32_t entry_count = (rsdt->header.length - sizeof(acpi_sdt_header_t)) / sizeof(uint32_t);

	int ret = 0;
	uint32_t i;
	for(i = 0; i < entry_count && ret == 0; i++)
	{
		/* Get current entry and map it */
		acpi_sdt_header_t* entry = (acpi_sdt_header_t*)rsdt->entry[i];
		kmmap_identity((size_t) entry, VMA_RW);

		/* Cast signature for easier output */
		char* sig = (char*) &entry->signature;

		if(acpi_checksum(entry, entry->length) != 0)
		{
			kprintf("ACPI table '%c%c%c%c' has incorrect checksum\n", sig[0], sig[1], sig[2], sig[3]);
			continue;
		}

		switch(entry->signature)
		{
		case MADT_SIGNATURE:
			acpi_madt = (acpi_madt_t*) entry;
			ret = parse_madt(acpi_madt);
			break;

		default:
			// not yet implemented
			//kprintf("Found table '%c%c%c%c', not yet implemented\n", sig[0], sig[1], sig[2], sig[3]);
			break;
		}
	}

	/* We have to find the MADT, otherwise fail */
	if(!acpi_madt)
		return -ENOENT;

	return 0;
}

// ----------------------------------------------------------------------------

acpi_rsdt_t*
get_acpi_rsdt(void)
{
	return acpi_rsdt;
}

// ----------------------------------------------------------------------------

acpi_madt_t*
get_acpi_madt(void)
{
	return acpi_madt;
}

// ----------------------------------------------------------------------------

acpi_rsdp_t*
get_acpi_rsdp(void)
{
	if(acpi_rsdp)
		return acpi_rsdp;

	if( (acpi_rsdp = search_rdsp(EBDA_ADDRESS, EBDA_LIMIT)) )
		return acpi_rsdp;

	if( (acpi_rsdp = search_rdsp(BIOS_ROM_ADDRESS, BIOS_ROM_LIMIT)) )
		return acpi_rsdp;

	// No RSDP found
	return NULL;
}

// ----------------------------------------------------------------------------

/*
 * @brief Search ACPI structures and scan for relevant tables
 *
 * TODO: Maybe parse XSDT instead of RSDT
 */
int
acpi_init()
{
	if(!get_acpi_rsdp())
		return -ENOENT;

	kprintf("Host supports ACPI rev. %u.0\n", acpi_rsdp->revision + 1);

	/* Get RSDT table and map page containing it */
	acpi_rsdt = (acpi_rsdt_t*) acpi_rsdp->rsdt_adr;
	kmmap_identity((size_t) acpi_rsdt, VMA_RW);

	/* Validate checksum of RSDT */
	if(acpi_checksum(&acpi_rsdt->header, acpi_rsdt->header.length) != 0)
	{
		kputs("Bad RSDT checksum!\n");
		acpi_rsdt = NULL;
		return -EBADMSG;
	}

	/* This will also parse recursively all other ACPI tables */
	return parse_rsdt(acpi_rsdt);
}
