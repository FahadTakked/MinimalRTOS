// RTOS Framework - Spring 2019
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
//*******************Debug and Code test defines**********************//
#ifndef DEBUG
//#define DEBUG
#endif


//****************** Bit Banding defines for Pins *********************// (for TM4C123GXL-TIVA-C LAUNCHPAD)

//PORT E
#define PE1               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define PE2               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define PE3               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define PE4               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

//Port F
#define PF1               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define PF2               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PF3               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PF4               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

//Port A
#define PA2               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PA3               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PA4               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PA5               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PA6               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PA7               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))



//***************************** Board Modules Pins *************************// (for TM4C123GXL-TIVA-C LAUNCHPAD)

#define ONBOARD_RED_LED           PF1
#define ONBOARD_BLUE_LED          PF2
#define ONBOARD_GREEN_LED         PF3
#define ONBOARD_PUSH_BUTTON       PF4

//************************ External Modules Pins ***************************// (for EXTERNAL TESTBENCH)

#define RED_LED                   PE1
#define BLUE_LED                  ONBOARD_BLUE_LED
#define GREEN_LED                 PE2
#define YELLOW_LED                PE3
#define ORANGE_LED                PE4

#define PUSH_BUTTON_0             PA2
#define PUSH_BUTTON_1             PA3
#define PUSH_BUTTON_2             PA4
#define PUSH_BUTTON_3             PA5
#define PUSH_BUTTON_4             PA6

#define BUTTON_L1                 PUSH_BUTTON_4
#define BUTTON_L2                 PUSH_BUTTON_3

#define BUTTON_R1                 PUSH_BUTTON_2
#define BUTTON_R2                 PUSH_BUTTON_1
#define BUTTON_R3                 PUSH_BUTTON_0


//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

struct semaphore *SemaphorePt;                         // Pointer to SemaphorePt Semaphore, declared in kernel space


// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread


// Loop Variables
uint8_t i, j, k  = 0;                                  // Used in svcisr for loop

//
void* SystemStackPt;

uint8_t svc_value = 0;

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint8_t skips;
} tcb[MAX_TASKS];



// RTOS scheduler Data Structure
struct osScheduler
{
    uint8_t preemptiveEnable;                          // For Setting the Preemptive Scheduler Enable Mode
    uint8_t priorityEnable;                            // For Setting the Priority Scheduler Enable Mode
    uint8_t priorityInherit;                           // For Enabling Priority Inheritance
};
struct osScheduler scheduler;                          // Structure Variable to osScheduler Structure

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

    //TODO if preemptiveEnable = 0, then the scheduler hangs, correct this if you can.
    scheduler.priorityEnable   = 1;                                                            // Enable Condition for Priority Scheduler
    scheduler.preemptiveEnable = 1;                                                            // Enable Condition for Preemption


    // REQUIRED: initialize systick for 1ms system timer
    NVIC_ST_CTRL_R     = 0;                                                                    // Clear Control bit for safe programming
    NVIC_ST_CURRENT_R  = 0;                                                                    // Start Value
    NVIC_ST_RELOAD_R  |= 0x00009C3F;                                                               // Set for 1Khz, (40000000/1000) - 1
}


// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;

    ok = false;

    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;

        // Priority Scheduling
        if(scheduler.priorityEnable == 1)                                                        // Check if Priority condition is enabled (Default)
        {
            if(tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)                  // If yes and tasks are read and in UNRUN states
            {
                if(tcb[task].skips < tcb[task].currentPriority)                                  // Increment skip counts till the time it doesn't become equal to priority value
                {
                    tcb[task].skips++;                                                           // Increment skips
                    ok = false;                                                                  // Optional, for better readability
                }
                else if(tcb[task].skips >= tcb[task].currentPriority)                            // If greater than equal to current priority value
                {
                    tcb[task].skips = 0;                                                         // Clear Skip count
                    ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);     // Update value of 'ok' to get of the loop
                }
            }
        }
        // Round-Robin Scheduling
        else                                                                                     // In Else condition it goes round-robin where everyone gets a fair chance
        {
            tcb[task].skips = 0;                                                                 // Clear Skip counts if previously coming from Priority Scheduler
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);             // Update value of 'ok' to get of the loop
        }

    }

    return task;
}


// Function to set SP to desired pointer value
void setStackPt(void* func_stack)
{

    __asm(" MOV SP, R0"    );
    __asm(" BX LR"         );

}

// Get address of current SP
void* getStackPt()
{

    __asm(" MOV R3,SP"     );
    __asm(" ADD R3, #4"    );
    //    __asm(" BX  LR"        );

    // return 0;
}





void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    _fn fn;

    SystemStackPt = getStackPt();

    taskCurrent = rtosScheduler();

    setStackPt(tcb[taskCurrent].sp);

    fn = (_fn)tcb[taskCurrent].pid;

    tcb[taskCurrent].state = STATE_READY;

    NVIC_ST_CTRL_R |= 0x00000007;;      // set for source as clock interrupt enable and enable the timer.

    (*fn)();
    // Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, const char *name, int priority)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;

    priority = priority + 8;

    // REQUIRED: store the thread name
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].skips = 0;                                   // Initial skip count is 0 for all tasks
            strncpy(tcb[i].name,name, sizeof(tcb[i].name)-1);                        // Store the name of the task
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #100");

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #101");                                          // Execute SVC instruction for Service call, triggers svCallIsr()
}



// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #102");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #103");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{

    uint8_t tsk = 0;                                                      // Variable for task number loop

    // Preemptive scheduler support
    // Make sure there is at least one task with READY state as this is the first ISR to run,
    // in rtosInit() or will switch tasks that don't exist and will get into fault ISR.
    if(scheduler.preemptiveEnable == 1)
    {
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    }

    // Sleep function support
    for(tsk=0; tsk < MAX_TASKS; tsk++)
    {
        if(tcb[tsk].state == STATE_DELAYED)                               // Check If state is DELAYED
        {

            if(tcb[tsk].ticks > 0)
                tcb[tsk].ticks--;                                         // Decrement ticks only if ticks are greater than 0

            //if(tcb[tsk].ticks == 0)                                     // If ticks are equal to zero then make state READY
            else
                tcb[tsk].state = STATE_READY;
        }
    }

}

volatile uint32_t* PC_VAL = 0;
//uint32_t* LR_VAL

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm(" PUSH {r5,r6}");
    __asm(" PUSH {R8-R11}");

    tcb[taskCurrent].sp = getStackPt();                                   // save stack pointer in tcb
    setStackPt(SystemStackPt);                                            // set stack pointer to System Stack pointer

    taskCurrent = rtosScheduler();

    if(tcb[taskCurrent].state == STATE_READY)                             // If in Ready State
    {
        setStackPt(tcb[taskCurrent].sp);                                  // Set Thread SP to Processor SP

        __asm(" POP {r5,r6}");
        __asm(" POP {R8-R11}");

    }

    else if(tcb[taskCurrent].state == STATE_UNRUN)                        // If in Unrun State, give task a new stack, as if its already is running
    {
        setStackPt(tcb[taskCurrent].sp);

        __asm(" MOV R1, #0x01000000" );                                   // 0x01000000. XPSR
        __asm(" PUSH {R1}"           );                                   // Push XPSR
        PC_VAL = tcb[taskCurrent].pid;                                    // PC value, PC at thread, PID of thread
        __asm(" PUSH {R3}"           );                                   // Push PC
        __asm(" PUSH {LR}"           );                                   // Push LR
        __asm(" PUSH {R12}"          );                                   // Push R12
        __asm(" PUSH {R0-R3}"        );                                   // Push R0 to R3

        __asm(" MOVW R8, #0xFFF9"    );                                   // Move LSB to R8 register, adjusted as per compiler pushes
        __asm(" MOVT R8, #0xFFFF"    );                                   // Move MSB to R8 register
        __asm(" PUSH {R8}"           );                                   // Push value of LR, saved in R8, (hard coded) or saved at starting of pendSvIsr

        __asm(" PUSH {R7}"           );                                   // Push compiler register
        __asm(" PUSH {R4}"           );                                   // Push compiler register
        __asm(" PUSH {R3}"           );                                   // Push compiler register
        tcb[taskCurrent].state == STATE_READY;

    }

}

uint8_t get_svcValue(void)
{
    __asm(" MOV  R0, SP"   );                                             // Move SP to R0,     (address)
    __asm(" ADD  R0, #36"  );                                             // ADD offset   ,     (address)
    __asm(" LDR  R0, [R0]" );                                             // Load offset to R0  (value)
    __asm(" SUBS R0, #2"   );                                             // Subtract 2 from R0 (value), 2 byte offset of SVC instruction
    __asm(" LDR  R3, [R0]" );                                             // Load value to R0   (value)

    //  return 0;
}



// Empty function for returning the value of R0, 1st Argument to function
static uint32_t ret_R1(void)
{
    __asm(" MOV R1,R0");


}

static uint32_t ret_Rn(void)
{
    __asm(" MOV R0,R1"      );

}


// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    register uint32_t R1_ret __asm("r1");
    ret_R1();

    svc_value = get_svcValue();


    switch(svc_value)
    {

    case 100:
        tcb[taskCurrent].state = STATE_READY;

        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;

    case 101:
        tcb[taskCurrent].ticks = R1_ret;                                                  // Set sleep timeout value
        tcb[taskCurrent].state = STATE_DELAYED;                                           // Set state as delayed, it can't be scheduled till the time it is not in ready state

        NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV;                                           // Set pendsv bit to call pendsvISR for task switch
        break;

    case 102:
        SemaphorePt = (struct semaphore*)R1_ret;

        if(SemaphorePt->count > 0)                                                         // Check for value of semaphore count variable
        {
            SemaphorePt->count--;                                                          // Decrement the count if count > 0
            tcb[taskCurrent].semaphore = NULL;
            tcb[taskCurrent].semaphore = SemaphorePt;                                      // Must record if you are using it
        }
        else
        {
            SemaphorePt->processQueue[SemaphorePt->queueSize] =                            // Store task in semaphore process queue
                    (uint32_t)tcb[taskCurrent].pid;

            SemaphorePt->queueSize++;                                                      // Increment the index of the queue for next task

            tcb[taskCurrent].state     = STATE_BLOCKED;                                    // Mark the state of of current task as blocked
            tcb[taskCurrent].semaphore = NULL;                                             // Clear Before storing
            tcb[taskCurrent].semaphore = SemaphorePt;                                      // Store the pointer to semaphore, record the semaphore

            if(scheduler.priorityInherit == 1)                                             // Priority Inheritance Support
            {
                for(i=0; i<MAX_TASKS; i++)                                                 // Find previous user of this semaphore
                {
                    if(tcb[i].semaphore == tcb[taskCurrent].semaphore)                     // check who else uses the same semaphore, while current task is blocked
                    {
                        if(tcb[i].currentPriority > tcb[taskCurrent].currentPriority)      // if found then check if priority value is greater than task current
                        {
                            tcb[i].currentPriority = tcb[taskCurrent].currentPriority;     // if yes, then give the priority value of task current(lower) to other user
                        }
                    }
                }
            }

            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                      // Set pendsv Inside 'else' since we don't have switch task all the time
        }
        break;

    case 103:
        SemaphorePt = (struct semaphore*)R1_ret;                                               // Get Pointer to the semaphore, passed as argument in post()
        SemaphorePt->count++;                                                              // Increment the count, for other task to use resource
        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;                      // Restore Priority

        if(SemaphorePt->queueSize > 0)                                                     // someone is waiting in the semaphore queue
        {
            for(j = 0; j < MAX_TASKS; j++)
            {
                if(SemaphorePt->processQueue[0] == (uint32_t)tcb[j].pid)                   // Check if a task is waiting in the same sem queue
                {
                    SemaphorePt->processQueue[0] = 0;                                      // Release Task waiting in queue

                    tcb[j].state = STATE_READY;                                            // Make state ready of released task
                    SemaphorePt->count--;                                                  // Decrement the count, no two or more task should use the same resource

                    for(i = 0; i < SemaphorePt->queueSize; i++)
                    {
                        SemaphorePt->processQueue[i] =
                                SemaphorePt->processQueue[i+1];                            // Shift Semaphore process queue up

                    }

                    SemaphorePt->queueSize --;                                             // Decrement Queue Size

                    break;
                }
            }
        }

        //NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                          // Optional, call pendSvISR to switch task
        break;


    default:
        break;


    }

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    //******************************************************* Clock Configs ******************************************************************//

    // Configure System clock as 40Mhz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (0x04 << SYSCTL_RCC_SYSDIV_S);

    // Enable GPIO port A, and F peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE;



    //**************************************************** On Board Modules ******************************************************************//

    // Configure On boards RED, GREEN and BLUE led and Pushbutton Pins
    GPIO_PORTF_DEN_R |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);                  // Enable Digital
    GPIO_PORTF_DIR_R |= (1 << 1) | (1 << 2) | (1 << 3);                             // Enable as Output
    GPIO_PORTF_DIR_R &= ~(0x10);                                                    // Enable push button as Input
    GPIO_PORTF_PUR_R |= 0x10;                                                       // Enable internal pull-up for push button


    //    //********************************************** Console IO Hardware Configs *************************************************************//
    //
    //    // Configure UART0 pins
    //    SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R0;                                       // Turn-on UART0, leave other uarts in same status
    //    GPIO_PORTA_DEN_R   |= 3;                                                        // Turn on Digital Operations on PA0 and PA1
    //    GPIO_PORTA_AFSEL_R |= 3;                                                        // Select Alternate Functionality on PA0 and PA1
    //    GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;                  // Select UART0 Module
    //
    //    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    //    UART0_CTL_R   = 0;                                                              // turn-off UART0 to allow safe programming
    //    UART0_CC_R   |= UART_CC_CS_SYSCLK;                                              // use system clock (40 MHz)
    //    UART0_IBRD_R  = 21;                                                             // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    //    UART0_FBRD_R  = 45;                                                             // round(fract(r)*64)=45
    //    UART0_LCRH_R |= UART_LCRH_WLEN_8 | UART_LCRH_FEN;                               // configure for 8N1 w/ 16-level FIFO
    //    UART0_CTL_R  |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;                  // enable TX, RX, and module


    //***************************************************** External Modules ******************************************************************//

    // External Push Buttons
    GPIO_PORTA_DEN_R |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);       // Enable Digital for PORTA pins
    GPIO_PORTA_DIR_R &= ~(1 << 2) | ~(1 << 3) | ~(1 << 4) | ~(1 << 5) | ~(1 << 6);  // Clear Direction Registers, to make them Inputs
    GPIO_PORTA_PUR_R |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6);       // Enable Internal Pull-up for push buttons

    // External LEDs
    GPIO_PORTE_DEN_R |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);                  // Make PORTE Pins Digital
    GPIO_PORTE_DIR_R |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);                  // Set direction register to make them input

    //******************************************************* Systick Timer for Measurement ****************************************************//

    //    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;                                      // Enable Systick timer
    //    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                                // Disable timer before configuring (safe programming)
    //    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                                          // Configure as 32-bit timer (A+B)
    //    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;                     // Configure for periodic mode and Count Up timer

}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
// Function to get a return value from buttons pushed in OS test bench Hardware.
// retval: Value of Push Button pressed
uint8_t readPbs(void)
{
    uint8_t retval_button = 0;

    if(!PUSH_BUTTON_0)
        retval_button |= 1;

    if(!PUSH_BUTTON_1)
        retval_button |= 2;

    if(!PUSH_BUTTON_2)
        retval_button |= 4;

    if(!PUSH_BUTTON_3)
        retval_button |= 8;

    if(!PUSH_BUTTON_4)
        retval_button |= 16;

    return retval_button;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------






// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(2000);
        ORANGE_LED = 0;
        yield();
    }
}


//void idle2()
//{
//    while(true)
//    {
//        YELLOW_LED = 1;
//        //waitMicrosecond(100000);
//        YELLOW_LED = 0;
//        yield();
//    }
//}



void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
            scheduler.preemptiveEnable ^= 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{
    while (true)
    {
        // REQUIRED: add processing for the shell commands through the UART here
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    RED_LED = 1;
    waitMicrosecond(500000);
    RED_LED = 0;
    waitMicrosecond(500000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);
    //ok &=  createThread(idle2, "Idle2", 7);

    //    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
    ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(uncooperative, "Uncoop", 2);
    //    ok &= createThread(shell, "Shell", 0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
