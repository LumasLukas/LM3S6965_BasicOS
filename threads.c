#include <stdio.h>
#include "scheduler.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"

#include "inc/lm3s6965.h"
#include "rit128x96x4.h"



// **********************************************
// Global vars for Inter-thread communication
// ********************************************
volatile unsigned int LED_flag;

void thread1(void)
{
	while(1){
	  int c;
    switch( (c = getchar()) ) { 
      case EOF: // No characters are available
        clearerr(stdin);  // Must be used to clear EOF condition
        break;

      case '0':
			  if (lock_acquire(&threadlock)) {
		      iprintf("Uart1 thread: pressed 0, test\r\n");
		      lock_release(&threadlock);
		    }
			
        break;
			case '1':
			  if (lock_acquire(&threadlock)) {
  				iprintf("Uart1 Thread: pressed 1: toggle LED\r\n");
		      lock_release(&threadlock);
		    }
				LED_flag = 1;
				break;
      default:
        iprintf("Press 0, 1, 2, 3\r\n");
        break;  
				}
	}
	//}
	/*
    if (lock_acquire(&threadlock)) {
      // Simulate code that is occasionally interrupted
      iprintf("THIS IS T");
          yield(); // context switch "interrupt"
      iprintf("HREAD NU");
          yield(); // context switch "interrupt"
      iprintf("MBER 1\r\n");

      lock_release(&threadlock);
    }
	*/
}

  
void thread2(void)
{

  while (1) {
		if(LED_flag == 1){
			LED_flag = 0;//clear the flag
			GPIO_PORTF_DATA_R ^= 0x01;
		}

    //yield();
  }
}

///* this thread will contorl the OLED
// It will simply write a string to the OLED
// and then the thread will end
//
//*/
void thread3(void)
{
		// 
	  // OLED init
	  //
    RIT128x96x4Init(1000000);
    RIT128x96x4StringDraw("Scheduler Demo",       20,  0, 15);
}

void idleThread(void)
{
	while(1){		
		yield();
	}
}

void uart_thread2(void){
	volatile int i = 0;
	while(1){	
    
		for(i = 0; i < 200000; i++){}//delay
		
		if (lock_acquire(&threadlock)) {
		  iprintf("Uart2 thread: hello\r\n");
		  lock_release(&threadlock);
		}
	}
}
