/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   Copyright (c) 2020 Rafael Roman Otero.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
**/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: rpi-smartstart.c											}
{       Copyright(c): Leon de Boer(LdB) 2017								}
{       Version: 2.02														}
{																			}
{***************[ THIS CODE IS FREEWARE UNDER CC Attribution]***************}
{																            }
{     This sourcecode is released for the purpose to promote programming    }
{  on the Raspberry Pi. You may redistribute it and/or modify with the      }
{  following disclaimer and condition.                                      }
{																            }
{      The SOURCE CODE is distributed "AS IS" WITHOUT WARRANTIES AS TO      }
{   PERFORMANCE OF MERCHANTABILITY WHETHER EXPRESSED OR IMPLIED.            }
{   Redistributions of source code must retain the copyright notices to     }
{   maintain the author credit (attribution) .								}
{																			}
{***************************************************************************}
{                                                                           }
{      This code provides a 32bit or 64bit C wrapper around the assembler   }
{  stub code. In AARCH32 that file is SmartStart32.S, while in AARCH64 the  }
{  file is SmartStart64.S.													}
{	   There file also provides access to the basic hardware of the PI.     }
{  It is also the easy point to insert a couple of important very useful    }
{  Macros that make C development much easier.		        				}
{																            }
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
OUTPUT_ARCH(aarch64)
ENTRY(_start)
   /* Specify the memory areas */
   MEMORY
   {
         RAM (xrw)      : ORIGIN = 0x80000 , LENGTH = 0x8000000 /*128MB which is half memory on smallest memory PI, assumes GPU has other half */
    }

SECTIONS
{
.text :
    {
        . = ALIGN(4);
        __text_start__ = .;
        _start = .;
        KEEP(*(.text.startup))     /* This is what the startup code block is called in smartstart64.S */
        *(.text .text.* .gnu.linkonce.t.*)
        *(.rel.text .rel.text.* .rel.gnu.linkonce.t.*)
        *(.init .init.*)
        . = ALIGN(4);
        __text_end__ = .;
    } > RAM

.bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss .bss.* .gnu.linkonce.b.*)
        *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
   } > RAM

.rodata :
    {
        . = ALIGN(4);
        __rodata_start__ = .;
        *(.rodata .rodata.*)
        *(.rel.rodata .rel.rodata.* .rel.gnu.linkonce.r.*)
        . = ALIGN(4);
        __rodata_end__ = .;
    } > RAM

.data :
    {
        . = ALIGN(4);      /* Normal data memory is align 8 */
        __data_start__ = .;
        *(.data .data.* .gnu.linkonce.d.*)
        *(.rel.data .rel.data.* .rel.gnu.linkonce.d.*)
        . = ALIGN(4);
        __data_end__ = .;
   } > RAM

.data1 :
    {
        . = ALIGN(16);      /* data 1 is align 16 data needed for mailbox exchanges */
        __data1_start__ = .;
        *(.data1 .data1.*)
        *(.rel.data1 .rel.data1.*)
        . = ALIGN(16);
        __data1_end__ = .;
   } > RAM

.stack :
    {
        . = ALIGN(16);  /* Stack must always be aligned to 8 byte boundary AAPCS64 call standard */
        __stack_start__ = .;
        . = . + 512;    /* EL0 stack size */
        __EL0_stack_core0 = .;
        . = . + 32768;    /* EL1 stack size */
        __EL1_stack_core0 = .;
        . = . + 16384;  /* EL2 stack size (start-up) */
        __EL2_stack_core0 = .;
        . = ALIGN(16);
        __stack_end__ = .;

    } > RAM

.stack_core1 :
    {
        . = ALIGN(16);  /* Stack must always be aligned to 8 byte boundary AAPCS64 call standard */
        __stack_start_core1__ = .;
        . = . + 512;    /* EL0 stack size */
        __EL0_stack_core1 = .;
        . = . + 512;    /* EL1 stack size */
        __EL1_stack_core1 = .;
        . = . + 4096;  /* EL2 stack size (start-up) */
        __EL2_stack_core1 = .;
        . = ALIGN(16);
        __stack_end_core1__ = .;

    } > RAM

.stack_core2 :
    {
        . = ALIGN(16);  /* Stack must always be aligned to 8 byte boundary AAPCS call standard */
        __stack_start_core2__ = .;
        . = . + 512;    /* EL0 stack size */
        __EL0_stack_core2 = .;
        . = . + 512;    /* EL1 stack size */
        __EL1_stack_core2 = .;
        . = . + 4096;  /* EL2 stack size (start-up) */
        __EL2_stack_core2 = .;
        . = ALIGN(16);
        __stack_end_core2__ = .;

    } > RAM

.stack_core3 :
    {
        . = ALIGN(16);  /* Stack must always be aligned to 8 byte boundary AAPCS call standard */
        __stack_start_core3__ = .;
        . = . + 512;    /* EL0 stack size */
        __EL0_stack_core3 = .;
        . = . + 512;    /* EL1 stack size */
        __EL1_stack_core3 = .;
        . = . + 4096;  /* EL2 stack size (start-up) */
        __EL2_stack_core3 = .;
        . = ALIGN(16);
        __stack_end_core3__ = .;

    } > RAM

.ARM.attributes 0 : { *(.ARM.attributes) }

.Heap :
    {
     . = ALIGN(4);
     __heap_start__ = .;
     __end__ = .; PROVIDE ( __end__ = . );
     _end = .; PROVIDE (end = .); /* Any memory from here is free to use so this is end of code and start of heap */
    } > RAM

    /* Remove information from the standard libraries */
    /DISCARD/ :
    {
        /* We cant debug */

        /* so no point to hold arm unwind tables */
        *(.ARM.exidx .ARM.exidx.*)
        *(.rel.ARM.exidx .rel.ARM.exidx.* .gnu.linkonce.armexidx.*)

         /* same with debug frames */
        *(.debug_frame .debug_frame.*)

        /* We really don't need comments from compiler either  */
        *(.comment *.comment.*)

        /* We really don't need the symbols and string tables  */
        *(.symtab .symtab.*)
        *(.strtab .strtab.*)
        *(.shstrtab .shstrtab.*)

        /* remove any standard library hang over bits */
          libc.a ( * )
          libm.a ( * )
          libgcc.a ( * )
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.gnu.lto_*)
    }


}
