#include <stdio.h>
#include <setjmp.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "rit128x96x4.h"
#include "scheduler.h"

#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/lm3s6965.h"

//this is the address of a register that controls the sysTimer
//peripheral, used to trigger a timer interrupt
#define NVIC_INT_CTRL (*((volatile unsigned long *) 0xE000ED04))

//how many times the clock will click before resetting
//8000000 ticks is one second, so, /1000 is one milisecond
#define SYS_TIMER_RELOAD 8000000 / 1000 
#define STACK_SIZE 4096   // Amount of stack space for each thread


void reg_state_save(unsigned* savedregs, char* stack);
void reg_state_restore(unsigned* savedregs, char* stack);

typedef struct {
  int active;       // non-zero means thread is allowed to run
  char *stack;      // pointer to TOP of stack (highest memory location)
	unsigned savedregs[40]; //room for 10 registers new	
} threadStruct_t;

// thread_t is a pointer to function with no parameters and
// no return value...i.e., a user-space thread.
typedef void (*thread_t)(void);

// These are the external user-space threads. In this program, we create
// the threads statically by placing their function addresses in
// threadTable[]. A more realistic kernel will allow dynamic creation
// and termination of threads.
extern void thread1(void);
extern void thread2(void);
extern void thread3(void);
extern void idleThread(void);
extern void uart_thread2(void);
static thread_t threadTable[] = {
  thread1,
  thread2,
	thread3,
	idleThread,
	uart_thread2
};

//calculates the number of threads based on the size of the threadTable
#define NUM_THREADS (sizeof(threadTable)/sizeof(threadTable[0]))

// These static global variables are used in scheduler(), in
// the yield() function, and in threadStarter()
//static jmp_buf scheduler_buf;   // saves the state of the scheduler (replaced with our own custom function)
static threadStruct_t threads[NUM_THREADS]; // the thread table
unsigned currThread = -1;    // The currently active thread


// This is the lock variable used by all threads. Interface functions
// for it are:
//      void lock_init(unsigned *threadlockptr);        
//      unsigned lock_acquire(unsigned *threadlockptr);
//      void lock_release(unsigned *threadlockptr);     
unsigned threadlock;

/*
 * Inits the lock to an unlocked state
 */
void lock_init(unsigned *lock)
{
	    *lock = 1; //set the lock to initially be unlocked
}

/*
 * Used to aquire a lock.
 * returns 1 if we got the lock, 0 else.
 */
unsigned lock_acquire(unsigned *lock)
{
		asm volatile(
		             "MOV r1, #0\n" 
		
                 "LDREX r2, [r0]\n" 
                 "CMP r2, r1\n"
                 "ITT NE\n"
							   "STREXNE r2, r1, [r0]\n"
								 
							   "CMPNE r2, #1\n"
							   "BEQ 1f\n"
								 
							   "MOV r0, #1\n"
							   "BX LR\n"
								 
							   "1:\n"
							   "CLREX\n"
							   "MOV r0, #0\n"
							   "BX LR\n");
     
}

/*
 * Used to release a lock 
 */
void lock_release(unsigned *lock)
{
  asm volatile("MOV r1, #1\n"
							 "STR r1, [r0]\n"
	             //"STREXNE r2, r1, [r0]\n"
							 "BX LR\n");
}

/*
 * Simply returns the value of the control register.
 * This is used for debugging so we can see 
 * what mode we are in as well as what stack is 
 * being used.
 */
unsigned int readControlReg(){
	asm volatile( "MRS R0, CONTROL\n");
	return 0; //just to get rid of compiler error
}

/*
  changes privilige mode to unpriviliged as well
  as changes the stack being used to the process stack
*/
void priv_to_unpriv(void){
  asm volatile("MRS R3, CONTROL\n"
  //just changes to unpriv"ORR R3, R3, #1\n"
	"ORR R3, R3, #3\n"
  "MSR CONTROL, R3\n"
  "ISB");
}

/*
  changes privilige mode to priviliged as well
  as changes the stack being used to the Main Stack
*/
void unpriv_to_priv(void){
  asm volatile("MRS R3, CONTROL\n"
  "MOV R3, #0\n" //not good practice to set the entire reg to 0
  "MSR CONTROL, R3\n"
  "ISB");
}

/*
 * SVC used to trigger a timer interrupt when a thread calls yeild()
 * SVC needed because the threads are not in priviliged mode
 * and therefore cannot set the CTRL register.
 * Because SVC is an exception, it forces the system into 
 * priviliged mode and can set the bit needed to trigger and interupt
 */ 
void handleSVC(int code)
{	
  //triggers a timer interrupt by setting the PENDSTSET bit in
	//the CTRL register
	NVIC_INT_CTRL |=  NVIC_INT_CTRL_PENDSTSET; 
}


/*
 * This handler extracts the 8 bit number passed with 
 * an SVC interrupt and passes it to handleSVC
 *
 * NOTE: this 8 bit number is not needed at the moment
 * but is good practice to extract and pass it in case it 
 * is needed in the future.
*/
void svcIntHandler(void){
	
	asm volatile("LDR R0, [R13, #24]\n" //increment SP by 24(5 words), store into r0
                "SUB R0, R0, #2\n"               
                "LDRH R0, [R0]\n" //load halfword into r0
				        "B handleSVC\n");
}


/*
 * This timer interrupt is the scheduler.
 * It will perform a context switch to the next
 * active thread 
*/
void SysTickISR(void){
  
	//if its not the first time in the schedular,
	//save state of current Thread on the array of 10 elements
	if (currThread != -1){	
	  reg_state_save(threads[currThread].savedregs ,threads[currThread].stack);
	}

	// yield() returns here. Did the thread that just yielded to us exit? If
  // so, clean up its entry in the thread table.
  if (! threads[currThread].active) {
    free(threads[currThread].stack - STACK_SIZE);
  }
	
  //identify the next active thread
  unsigned i;
  i = NUM_THREADS;
  do {
    // Round-robin scheduler
    if (++currThread == NUM_THREADS) {
      currThread = 0;
    }
    if (threads[currThread].active) {
      break;
    } else {
      i--;
    }
  } while (i > 0);
	
  //restore state of next thread
  //from the array of 10 elements
	reg_state_restore(threads[currThread].savedregs, threads[currThread].stack);
}


/*
 * This function is called from within user thread context. It executes
 * a jump back to the scheduler. When the scheduler returns here, it acts
 * like a standard function return back to the caller of yield().
 
 * The number after svc is irrelevent in this context
 */
void yield(void)
{
	asm volatile("svc #229"); //cause a timer interrupt 
}

/* This is the starting	 point for all threads. It runs in user thread
 * context using the thread-specific stack. The address of this function
 * is saved by createThread() in the LR field of the jump buffer so that
 * the first time the scheduler() does a longjmp() to the thread, we
 * start here.
 */
void threadStarter(void)
{
  // Call the entry point for this thread. The next line returns
  // only when the thread exits.
  (*(threadTable[currThread]))();

  // Do thread-specific cleanup tasks. Currently, this just means marking
  // the thread as inactive. Do NOT free the stack here because we're
  // still using it! Remember, this function runs in user thread context.
  threads[currThread].active = 0;

  // This yield returns to the scheduler and never returns back since
  // the scheduler identifies the thread as inactive.
  yield();
}


//save everything not automatically saved by the system
//will put the values of the 10 registers into saved regs
void reg_state_save(unsigned* savedregs, char* stack)
{
	
	asm volatile(
	             "mrs r1, psp\n" //move psp value into r1
							 "stm r0, {r1, r4-r12}\n" //store val from sp 
							 "bx lr\n"
	
	);
  return;
}

void reg_state_restore(unsigned* savedregs, char* stack)
{
	
	  //(Load multiple increment after)  
		//loads the registers with vals from savedregs 
		//r1, r4-r12 the psp was stored in r1 
  asm volatile("ldmia r0, {r1, r4-r12}\n"
               "msr psp, r1\n" //Move value in r1 to PSP
							 //Loading 0xFFFFFFFD into lr must switch back 
							 //to unpriv mode and process stack
							 //0b1101: return to thread mode, exception return gets its state from the process stack
               "ldr lr, =0xFFFFFFFD\n"
               "bx lr\n"); //Branch to Link Register
	
	return;
}




// This function is implemented in assembly language. It sets up the
// initial jump-buffer (as would setjmp()) but with our own values
// for the stack (passed to createThread()) and LR (always set to
// threadStarter() for each thread).
extern void createThread(unsigned* regs, char *stack);

int main(void)
{
  unsigned i;

  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_8MHZ);
								   
  // UART Init
	//enable uart periph
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Set GPIO A0 and A1 as UART pins.
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

	//LED Init
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
  GPIO_PORTF_DEN_R |= 0x01; //might have a problem with these lines
  GPIO_PORTF_DIR_R |= 0x01;

	// Create all the threads and allocate a stack for each one and set active
  for (i=0; i < NUM_THREADS; i++) {
    // Mark thread as runnable
    threads[i].active = 1;
    // Allocate stack
    threads[i].stack = (char *)malloc(STACK_SIZE) + STACK_SIZE;
    if (threads[i].stack == 0) {
      iprintf("Out of memory\r\n");
      exit(1);
    }
		//inits the stack and reg values of each thread (find in create.S)
    createThread(threads[i].savedregs, threads[i].stack);
  }
	
	//
	// Setting up the SysTimer (which will be the schedular)
	NVIC_ST_CTRL_R = 0x0;  //set to zero so not counting during setup
	//NVIC_ST_RELOAD_R for the SYST_RVR
	NVIC_ST_RELOAD_R = SYS_TIMER_RELOAD; // should be every milisecond(second0x1E8480; 2000000)
  //NVIC_ST_CURRENT_R for the current counter value
	NVIC_ST_CURRENT_R = 0x0;
	
	//use NVIC_ST_CTRL_R for SYST_CSR
	//CLKSOURCE=1 
	//ENABLE=1
	//TickINT = 1 (for interrupts)
	NVIC_ST_CTRL_R = 0x7; 
	
	//init the lock for the two uart threads
	lock_init(&threadlock);
	
	//Globally enable interrupts
  IntMasterEnable();	
  
	//will wait for the first timer interrupt where it will jump to the schedular
	while(1){}; 
	
  return 0;
}

/*
 * Compile with:
 * ${CC} -o lockdemo.elf -I${STELLARISWARE} -L${STELLARISWARE}/driverlib/gcc 
 *     -Tlinkscript.x -Wl,-Map,lockdemo.map -Wl,--entry,ResetISR 
 *     lockdemo.c create.S threads.c startup_gcc.c syscalls.c rit128x96x4.c 
 *     -ldriver
 */
// vim: expandtab ts=2 sw=2 cindent
