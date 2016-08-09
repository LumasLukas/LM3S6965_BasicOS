/*
 * Implement the thread creation task:
 *
 *   - initialize the jump buffer with appropriate values for
 *     R13 (stack) and R14 (first address to jump to)
 *   - all other registers are irrelevant upon thread creation
 *
 *   In the jump buffer, the R13 slot is set to the second parameter of this
 *   function (the top-of-stack address, passed in R1). The R14 slot is set to
 *   the address of the threadStarter() function.
 *
 *   The C prototype for this function call is:
 *       createThread(threads[i].state, threads[i].stack)
 *   thus:
 *       R0 <-- state (a setjmp()-style jump buffer)
 *       R1 <-- stack (address of top-of-stack)
 */
    .syntax unified
    .text
    .align 2
    .thumb
    .thumb_func
    .type createThread,function
    .global createThread
createThread:

  @ Push r4-r11 onto the stack
  push {r4-r11}

	
	
  @ Moving 0x01000000 into r12 (will later be xPSR)
  @0b0001000
	mov r12, #0x0000
  movt r12, #0x0100

  @ Loading threadStarter() function into r11
	@we will set this as A2R2
  ldr  r11, .L0

	@Store multiple decrement before
	@stores r5-r12 into mem location held by r1 (the threads stack)
	@only have values in r11,r12, but this will update the pointer
  @stack: xPSR,LR,R13,R12,R3,R2,R1,R0
	@now the thread function will return to the correct place
  stmdb r1!, {r5-r12}
	
	@store multiple empty assending
  @store values r1 r4-r12 onto the array or registers
	@the psp will have been updated from the previous instr
	@the rest of the registers values do not matter
  stmea r0!, {r1,r4-r12}
  @filling the regisers with default values
	
	
  @restore the stack to state before the function call
  pop {r4-r11}

  @ Branch to Link Register
  bx      lr


.L0:
    .word   threadStarter





