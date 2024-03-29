	.file "C:\\_dsmps\\DPSK3\\Troubleshooting\\__src__\\dpsk3-firmware\\pic24\\mcc_generated_files\\interrupt_manager.c"
	.section	.debug_abbrev,info
.Ldebug_abbrev0:
	.section	.debug_info,info
.Ldebug_info0:
	.section	.debug_line,info
.Ldebug_line0:
	.section	.text,code
.Ltext0:
	.section	.text,code
	.align	2
	.global	_INTERRUPT_Initialize	; export
	.type	_INTERRUPT_Initialize,@function
_INTERRUPT_Initialize:
.LFB0:
	.file 1 "mcc_generated_files/interrupt_manager.c"
	.loc 1 54 0
	.set ___PA___,1
	.loc 1 57 0
	mov	#-28673,w0
	mov	_IPC0bits,w1
	and	w0,w1,w2
	mov	#12288,w1
	ior	w1,w2,w1
	mov	w1,_IPC0bits
	.loc 1 60 0
	mov	#-113,w1
	mov	_IPC3bits,w3
	and	w1,w3,w2
	bset	w2,#6
	mov	w2,_IPC3bits
	.loc 1 63 0
	mov	_IPC16bits,w2
	and	w1,w2,w1
	bset	w1,#4
	mov	w1,_IPC16bits
	.loc 1 66 0
	mov	#-8,w1
	mov	_IPC3bits,w3
	and	w1,w3,w1
	bset	w1,#0
	mov	w1,_IPC3bits
	.loc 1 69 0
	and	_IPC2bits,WREG
	bset	w0,#13
	mov	w0,_IPC2bits
	.loc 1 71 0
	return	
	.set ___PA___,0
.LFE0:
	.size	_INTERRUPT_Initialize, .-_INTERRUPT_Initialize
	.section	.debug_frame,info
.Lframe0:
	.4byte	.LECIE0-.LSCIE0
.LSCIE0:
	.4byte	0xffffffff
	.byte	0x1
	.byte	0
	.uleb128 0x1
	.sleb128 2
	.byte	0x25
	.byte	0x12
	.uleb128 0xf
	.sleb128 -2
	.byte	0x9
	.uleb128 0x25
	.uleb128 0xf
	.align	4
.LECIE0:
.LSFDE0:
	.4byte	.LEFDE0-.LASFDE0
.LASFDE0:
	.4byte	.Lframe0
	.4byte	.LFB0
	.4byte	.LFE0-.LFB0
	.align	4
.LEFDE0:
	.section	.text,code
.Letext0:
	.file 2 "c:\\program files (x86)\\microchip\\xc16\\v1.36\\bin\\bin\\../..\\support\\PIC24F\\h/p24FJ64GA004.h"
	.file 3 "c:\\program files (x86)\\microchip\\xc16\\v1.36\\bin\\bin\\../..\\include\\lega-c/stdint.h"
	.section	.debug_info,info
	.4byte	0x7a5
	.2byte	0x2
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.uleb128 0x1
	.asciz	"GNU C 4.5.1 (XC16, Microchip v1.36) (B) Build date: Jan 25 2019"
	.byte	0x1
	.asciz	"mcc_generated_files/interrupt_manager.c"
	.ascii	"C:\\\\_dsmps\\\\DPSK3\\\\Troubleshooting\\\\__src__\\\\dpsk3-firmwar"
	.asciz	"e\\\\pic24"
	.4byte	.Ltext0
	.4byte	.Letext0
	.4byte	.Ldebug_line0
	.uleb128 0x2
	.byte	0x1
	.byte	0x6
	.asciz	"signed char"
	.uleb128 0x2
	.byte	0x2
	.byte	0x5
	.asciz	"int"
	.uleb128 0x2
	.byte	0x4
	.byte	0x5
	.asciz	"long int"
	.uleb128 0x2
	.byte	0x8
	.byte	0x5
	.asciz	"long long int"
	.uleb128 0x2
	.byte	0x1
	.byte	0x8
	.asciz	"unsigned char"
	.uleb128 0x3
	.asciz	"uint16_t"
	.byte	0x3
	.byte	0x31
	.4byte	0x118
	.uleb128 0x2
	.byte	0x2
	.byte	0x7
	.asciz	"unsigned int"
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.asciz	"long unsigned int"
	.uleb128 0x2
	.byte	0x8
	.byte	0x7
	.asciz	"long long unsigned int"
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x193
	.4byte	0x1b1
	.uleb128 0x5
	.asciz	"INT0IP"
	.byte	0x2
	.2byte	0x194
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0xd
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"IC1IP"
	.byte	0x2
	.2byte	0x196
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"OC1IP"
	.byte	0x2
	.2byte	0x198
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x5
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"T1IP"
	.byte	0x2
	.2byte	0x19a
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x1
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x19c
	.4byte	0x2b7
	.uleb128 0x5
	.asciz	"INT0IP0"
	.byte	0x2
	.2byte	0x19d
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xf
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"INT0IP1"
	.byte	0x2
	.2byte	0x19e
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xe
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"INT0IP2"
	.byte	0x2
	.2byte	0x19f
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xd
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"IC1IP0"
	.byte	0x2
	.2byte	0x1a1
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xb
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"IC1IP1"
	.byte	0x2
	.2byte	0x1a2
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xa
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"IC1IP2"
	.byte	0x2
	.2byte	0x1a3
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"OC1IP0"
	.byte	0x2
	.2byte	0x1a5
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x7
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"OC1IP1"
	.byte	0x2
	.2byte	0x1a6
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x6
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"OC1IP2"
	.byte	0x2
	.2byte	0x1a7
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x5
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"T1IP0"
	.byte	0x2
	.2byte	0x1a9
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x3
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"T1IP1"
	.byte	0x2
	.2byte	0x1aa
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x2
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"T1IP2"
	.byte	0x2
	.2byte	0x1ab
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x1
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x6
	.byte	0x2
	.byte	0x2
	.2byte	0x192
	.4byte	0x2cb
	.uleb128 0x7
	.4byte	0x157
	.uleb128 0x7
	.4byte	0x1b1
	.byte	0x0
	.uleb128 0x8
	.asciz	"tagIPC0BITS"
	.byte	0x2
	.byte	0x2
	.2byte	0x191
	.4byte	0x2e9
	.uleb128 0x9
	.4byte	0x2b7
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0xa
	.asciz	"IPC0BITS"
	.byte	0x2
	.2byte	0x1ae
	.4byte	0x2cb
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x1d3
	.4byte	0x356
	.uleb128 0x5
	.asciz	"T3IP"
	.byte	0x2
	.2byte	0x1d4
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0xd
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPF1IP"
	.byte	0x2
	.2byte	0x1d6
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPI1IP"
	.byte	0x2
	.2byte	0x1d8
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x5
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1RXIP"
	.byte	0x2
	.2byte	0x1da
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x1
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x1dc
	.4byte	0x462
	.uleb128 0x5
	.asciz	"T3IP0"
	.byte	0x2
	.2byte	0x1dd
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xf
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"T3IP1"
	.byte	0x2
	.2byte	0x1de
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xe
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"T3IP2"
	.byte	0x2
	.2byte	0x1df
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xd
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPF1IP0"
	.byte	0x2
	.2byte	0x1e1
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xb
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPF1IP1"
	.byte	0x2
	.2byte	0x1e2
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xa
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPF1IP2"
	.byte	0x2
	.2byte	0x1e3
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPI1IP0"
	.byte	0x2
	.2byte	0x1e5
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x7
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPI1IP1"
	.byte	0x2
	.2byte	0x1e6
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x6
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"SPI1IP2"
	.byte	0x2
	.2byte	0x1e7
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x5
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1RXIP0"
	.byte	0x2
	.2byte	0x1e9
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x3
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1RXIP1"
	.byte	0x2
	.2byte	0x1ea
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x2
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1RXIP2"
	.byte	0x2
	.2byte	0x1eb
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x1
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x6
	.byte	0x2
	.byte	0x2
	.2byte	0x1d2
	.4byte	0x476
	.uleb128 0x7
	.4byte	0x2fa
	.uleb128 0x7
	.4byte	0x356
	.byte	0x0
	.uleb128 0x8
	.asciz	"tagIPC2BITS"
	.byte	0x2
	.byte	0x2
	.2byte	0x1d1
	.4byte	0x494
	.uleb128 0x9
	.4byte	0x462
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0xa
	.asciz	"IPC2BITS"
	.byte	0x2
	.2byte	0x1ee
	.4byte	0x476
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x1f5
	.4byte	0x4d8
	.uleb128 0x5
	.asciz	"U1TXIP"
	.byte	0x2
	.2byte	0x1f6
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0xd
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"AD1IP"
	.byte	0x2
	.2byte	0x1f8
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x1fa
	.4byte	0x563
	.uleb128 0x5
	.asciz	"U1TXIP0"
	.byte	0x2
	.2byte	0x1fb
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xf
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1TXIP1"
	.byte	0x2
	.2byte	0x1fc
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xe
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1TXIP2"
	.byte	0x2
	.2byte	0x1fd
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xd
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"AD1IP0"
	.byte	0x2
	.2byte	0x1ff
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xb
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"AD1IP1"
	.byte	0x2
	.2byte	0x200
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xa
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"AD1IP2"
	.byte	0x2
	.2byte	0x201
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x6
	.byte	0x2
	.byte	0x2
	.2byte	0x1f4
	.4byte	0x577
	.uleb128 0x7
	.4byte	0x4a5
	.uleb128 0x7
	.4byte	0x4d8
	.byte	0x0
	.uleb128 0x8
	.asciz	"tagIPC3BITS"
	.byte	0x2
	.byte	0x2
	.2byte	0x1f3
	.4byte	0x595
	.uleb128 0x9
	.4byte	0x563
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0xa
	.asciz	"IPC3BITS"
	.byte	0x2
	.2byte	0x204
	.4byte	0x577
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x31d
	.4byte	0x5ee
	.uleb128 0x5
	.asciz	"U1ERIP"
	.byte	0x2
	.2byte	0x31f
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U2ERIP"
	.byte	0x2
	.2byte	0x321
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x5
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"CRCIP"
	.byte	0x2
	.2byte	0x323
	.4byte	0x108
	.byte	0x2
	.byte	0x3
	.byte	0x1
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x4
	.byte	0x2
	.byte	0x2
	.2byte	0x325
	.4byte	0x6bb
	.uleb128 0x5
	.asciz	"U1ERIP0"
	.byte	0x2
	.2byte	0x327
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xb
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1ERIP1"
	.byte	0x2
	.2byte	0x328
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0xa
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U1ERIP2"
	.byte	0x2
	.2byte	0x329
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x9
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U2ERIP0"
	.byte	0x2
	.2byte	0x32b
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x7
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U2ERIP1"
	.byte	0x2
	.2byte	0x32c
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x6
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"U2ERIP2"
	.byte	0x2
	.2byte	0x32d
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x5
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"CRCIP0"
	.byte	0x2
	.2byte	0x32f
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x3
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"CRCIP1"
	.byte	0x2
	.2byte	0x330
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x2
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.uleb128 0x5
	.asciz	"CRCIP2"
	.byte	0x2
	.2byte	0x331
	.4byte	0x108
	.byte	0x2
	.byte	0x1
	.byte	0x1
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0x6
	.byte	0x2
	.byte	0x2
	.2byte	0x31c
	.4byte	0x6cf
	.uleb128 0x7
	.4byte	0x5a6
	.uleb128 0x7
	.4byte	0x5ee
	.byte	0x0
	.uleb128 0x8
	.asciz	"tagIPC16BITS"
	.byte	0x2
	.byte	0x2
	.2byte	0x31b
	.4byte	0x6ee
	.uleb128 0x9
	.4byte	0x6bb
	.byte	0x2
	.byte	0x23
	.uleb128 0x0
	.byte	0x0
	.uleb128 0xa
	.asciz	"IPC16BITS"
	.byte	0x2
	.2byte	0x334
	.4byte	0x6cf
	.uleb128 0xb
	.byte	0x1
	.asciz	"INTERRUPT_Initialize"
	.byte	0x1
	.byte	0x35
	.byte	0x1
	.4byte	.LFB0
	.4byte	.LFE0
	.byte	0x1
	.byte	0x5f
	.uleb128 0xc
	.4byte	.LASF0
	.byte	0x2
	.2byte	0x1af
	.4byte	0x732
	.byte	0x1
	.byte	0x1
	.uleb128 0xd
	.4byte	0x2e9
	.uleb128 0xc
	.4byte	.LASF1
	.byte	0x2
	.2byte	0x1ef
	.4byte	0x745
	.byte	0x1
	.byte	0x1
	.uleb128 0xd
	.4byte	0x494
	.uleb128 0xc
	.4byte	.LASF2
	.byte	0x2
	.2byte	0x205
	.4byte	0x758
	.byte	0x1
	.byte	0x1
	.uleb128 0xd
	.4byte	0x595
	.uleb128 0xc
	.4byte	.LASF3
	.byte	0x2
	.2byte	0x335
	.4byte	0x76b
	.byte	0x1
	.byte	0x1
	.uleb128 0xd
	.4byte	0x6ee
	.uleb128 0xc
	.4byte	.LASF0
	.byte	0x2
	.2byte	0x1af
	.4byte	0x732
	.byte	0x1
	.byte	0x1
	.uleb128 0xc
	.4byte	.LASF1
	.byte	0x2
	.2byte	0x1ef
	.4byte	0x745
	.byte	0x1
	.byte	0x1
	.uleb128 0xc
	.4byte	.LASF2
	.byte	0x2
	.2byte	0x205
	.4byte	0x758
	.byte	0x1
	.byte	0x1
	.uleb128 0xc
	.4byte	.LASF3
	.byte	0x2
	.2byte	0x335
	.4byte	0x76b
	.byte	0x1
	.byte	0x1
	.byte	0x0
	.section	.debug_abbrev,info
	.uleb128 0x1
	.uleb128 0x11
	.byte	0x1
	.uleb128 0x25
	.uleb128 0x8
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x1b
	.uleb128 0x8
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x10
	.uleb128 0x6
	.byte	0x0
	.byte	0x0
	.uleb128 0x2
	.uleb128 0x24
	.byte	0x0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0x8
	.byte	0x0
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x16
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x4
	.uleb128 0x13
	.byte	0x1
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x5
	.uleb128 0xd
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0xd
	.uleb128 0xb
	.uleb128 0xc
	.uleb128 0xb
	.uleb128 0x38
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x6
	.uleb128 0x17
	.byte	0x1
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x7
	.uleb128 0xd
	.byte	0x0
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x8
	.uleb128 0x13
	.byte	0x1
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x9
	.uleb128 0xd
	.byte	0x0
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x38
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0xa
	.uleb128 0x16
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0xb
	.uleb128 0x2e
	.byte	0x0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0xc
	.uleb128 0x34
	.byte	0x0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3c
	.uleb128 0xc
	.byte	0x0
	.byte	0x0
	.uleb128 0xd
	.uleb128 0x35
	.byte	0x0
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.byte	0x0
	.section	.debug_pubnames,info
	.4byte	0x27
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x7a9
	.4byte	0x700
	.asciz	"INTERRUPT_Initialize"
	.4byte	0x0
	.section	.debug_pubtypes,info
	.4byte	0x91
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x7a9
	.4byte	0x108
	.asciz	"uint16_t"
	.4byte	0x2cb
	.asciz	"tagIPC0BITS"
	.4byte	0x2e9
	.asciz	"IPC0BITS"
	.4byte	0x476
	.asciz	"tagIPC2BITS"
	.4byte	0x494
	.asciz	"IPC2BITS"
	.4byte	0x577
	.asciz	"tagIPC3BITS"
	.4byte	0x595
	.asciz	"IPC3BITS"
	.4byte	0x6cf
	.asciz	"tagIPC16BITS"
	.4byte	0x6ee
	.asciz	"IPC16BITS"
	.4byte	0x0
	.section	.debug_aranges,info
	.4byte	0x14
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0x0
	.2byte	0x0
	.2byte	0x0
	.4byte	0x0
	.4byte	0x0
	.section	.debug_str,info
.LASF3:
	.asciz	"IPC16bits"
.LASF0:
	.asciz	"IPC0bits"
.LASF1:
	.asciz	"IPC2bits"
.LASF2:
	.asciz	"IPC3bits"
	.section	.text,code



	.section __c30_signature, info, data
	.word 0x0001
	.word 0x0000
	.word 0x0000

; MCHP configuration words

	.set ___PA___,0
	.end
