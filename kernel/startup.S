  .global Vector_Init

  .balign 4
  .code 32

	.equ T_bit, 0x20

// Extern Label
	.extern vPortSVCHandler
	.extern vPortResetHandler
	.extern Undefined_Handler_Panic
	.extern Prefetch_Handler_Panic
	.extern Abort_Handler_Panic
	.extern __stack0
	.extern __stack1
	.extern _init

// Exception Vectors
        .section .isr_vector, "x"

Vector_Init:													// Exception							Entry Mode
        LDR     PC, Reset_Addr				// Reset									SVC
        LDR     PC, Undefined_Addr		// undefined instruction	UNDEF
        LDR     PC, SWI_Addr					// SWI(SW Interrupt)			SVC
        LDR     PC, Prefetch_Addr			// Abort(prefetch)				ABT
        LDR     PC, Abort_Addr				// Abort(data)						ABT
        NOP														// Reserved								Reserved
        LDR     PC, IRQ_Addr					// IRQ										IRQ
        LDR     PC, FIQ_Addr					// FIQ										FIQ

Reset_Addr:      .word     Reset_Handler
Undefined_Addr:  .word     Undefined_Handler
SWI_Addr:        .word     SWI_Handler
Prefetch_Addr:   .word     Prefetch_Handler
Abort_Addr:      .word     Abort_Handler
IRQ_Addr:        .word     IRQ_Handler
FIQ_Addr:        .word     FIQ_Handler

// Exception Handlers
Reset_Handler:
	mrc		p15,0,r2,c0,c0,5
	and		r2,r2,#3					// Find Core ID

	mov		r1, #0
	cmp		r1, r2
	ble		CPU0_Config	
	b			CommonSection

CPU0_Config:
	mrc   p15,0,r1,c1,c0,0    // Read control register configuration data
	bic   r1,r1,#(1<<0)     	// Disable MMU
	bic   r1,r1,#(1<<12)      // Disable I Cache
	bic   r1,r1,#(1<<2)     	// Disable D Cache
	mcr   p15,0,r1,c1,c0,0    // Write control register configuration data

	ldr   r1,=0x48281800
	mov   r2,#0x0004
	str   r2,[r1]         		// AUX_CORE_BOOT_0

	ldr   r1,=0x48281804
	adr   r2,CommonSection
	str   r2,[r1]         		// AUX_CORE_BOOT_1

	dmb
	dsb
	sev
	nop

CommonSection:
	mrc		p15,0,r2,c0,c0,5
	and		r2,r2,#3						// Find Core ID

	adr		r1, Stack_Table
	ldr		SP, [r1,r2,lsl #2]	// r1 = r2<<2
	b			_init

Stack_Table:
	.word	__stack0
	.word __stack1

// ARM Architecture & Thumb C & C++ Compiler always use descending stack
Undefined_Handler:
    STMDB      SP, {R0-LR}^
    NOP
    SUB        SP, SP, #60
    MRS        R0, SPSR
    SUB        R1, LR, #2

    TST        R0, #T_bit
    SUBEQ      R1, R1, #2
    STMDB      SP!, {R1}
    STMDB      SP!, {R0}
    NOP
    MOV        R0, SP
    BL         Undefined_Handler_Panic
Prefetch_Handler:
    STMDB      SP, {R0-LR}^
    NOP
    SUB        SP, SP, #60
    SUB        R0, LR, #4
    STMDB      SP!, {R0}
    MRS        R0, SPSR
    STMDB      SP!, {R0}
    NOP
    MOV        R0, SP
    BL         Prefetch_Handler_Panic
Abort_Handler:
    STMDB      SP, {R0-LR}^
    NOP
    SUB        SP, SP, #60
    SUB        R0, LR, #8
    STMDB      SP!, {R0}
    MRS        R0, SPSR
    STMDB      SP!, {R0}
    NOP
    MOV        R0, SP
    BL         Abort_Handler_Panic

SWI_Handler:
		B				vPortSVCHandler

FIQ_Handler:
IRQ_Handler:
		B				vPortInterruptContext

    .end
