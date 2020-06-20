/* ------------------------------------------------------------------------------------------------ */
/* FILE :		ISRhandler.s																		*/
/*																									*/
/* CONTENTS :																						*/
/*				Interrupt dispatcher & interrupt enbling assembly code								*/
/*																									*/
/*																									*/
/* Copyright (c) 2013, Code-Time Technologies Inc. All rights reserved.								*/
/*																									*/
/* Code-Time Technologies retains all right, title, and interest in and to this work				*/
/*																									*/
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS							*/
/* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF										*/
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL							*/
/* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR								*/
/* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,							*/
/* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR							*/
/* OTHER DEALINGS IN THE SOFTWARE.																	*/
/*																									*/
/*																									*/
/*	$Revision: 1.8 $																				*/
/*	$Date: 2013/10/03 15:55:16 $																	*/
/*																									*/
/* ------------------------------------------------------------------------------------------------ */

	.code 32

	.text
	.align	2

	.global	G_OSisrTbl
	.global __cs3_isr_irq
	.type	__cs3_isr_irq, %function

__cs3_isr_irq:
	cps		#0x1F							/* Go into SYS mode to save IRQ context on user's stack	*/
	stmfd	sp!, {r0-r3, r10-r12, lr}		/* User stack if a task switch is trigger by this ISR	*/
	cps		#0x12							/* Go back to IRQ mode									*/
	mov		r11, lr							/* Preserve LR_irq (R11 was pushed on user's stack)		*/
	mrc 	p15, #4, r1, c15, c0, #0		/* Get the PERIPH_BASE_ADDR from P15 register			*/
	ldr		r0, [r1, #0x10C]				/* Read the ICCIAR register to get the interrupt #		*/
	str		r0, [r1, #0x110]				/* Inform the GIC the interrupt has been handled		*/
	mov		r0, r0, lsl #(31-9)				/* Keep the ACKINTID bits only (need << 2 and & MASK)	*/
	ldr		r1, =G_OSisrTbl					/* The function handler pointers are in G_OSisrTbl[]	*/
	mov		lr, pc							/* Set-up the return address							*/
	ldr		pc, [r1, r0, lsr #(31-11)]		/* Get the ISR handler function pointer and call it		*/
											/* ACKINTID times *4 to access the 32 bits array		*/
	mov		lr, r11							/* Put back LR_irq										*/
	cps		#0x1F							/* Go into SYS mode to recover what is on the stack		*/
	ldmfd	sp!, {r0-r3, r10-r12, lr}		/* Remainder of the context that was saved on SYS stack	*/
	cps		#0x12							/* Back to IRQ mode to exit								*/
	subs	pc, lr, #4						/* Exit the ISR											*/

	.size	__cs3_isr_irq, .-__cs3_isr_irq

/* ------------------------------------------------------------------------------------------------ */

	.text
	.align	2

	.global	__cs3_interrupt_vector
	.global	EnbInt
	.type	EnbInt, %function

EnbInt:
	ldr		r1, =__cs3_interrupt_vector
	mcr		p15, #0, r1, c12, c0, #0		/* Set the base address of the vector table in VBAR		*/

	mrs		r3, CPSR						/* Need to know if USER/SYS or other & if ISRs enable	*/
	bic		r2, r3, #0x80					/* Enable is IRQ bit clear								*/
	msrne	CPSR_c, r2						/* No need to disable as ISRs are disabled right now	*/
	mov		pc, lr

	.size	EnbInt, .-EnbInt

/* EOF */