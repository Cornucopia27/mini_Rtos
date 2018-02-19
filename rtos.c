/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				2
#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000

#define SP_OFFSET                   9

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum
{
	S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;
typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
} task_list =
{ 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
	task_list.global_tick = 0;  //sets global clock
	rtos_create_task(idle_task, 0, kAutoStart); //creates idle task with lowest priority
	task_list.current_task = RTOS_INVALID_TASK;
#endif
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;
	reload_systick();
	for (;;)
		;
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
    rtos_task_handle_t retval = RTOS_INVALID_TASK;
    //comparison to check if we already have the max number of tasks
	if(RTOS_MAX_NUMBER_OF_TASKS > task_list.nTasks)
	{
		task_list.tasks[task_list.nTasks].priority = priority; //assigns the priority level of the task
		task_list.tasks[task_list.nTasks].local_tick = 0; //sets the local tick
        task_list.tasks[task_list.nTasks].task_body = task_body;
        task_list.tasks[task_list.nTasks].sp =
                &(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - 1
                        - STACK_FRAME_SIZE]);
        task_list.tasks[task_list.nTasks].state =
                kStartSuspended == autostart ? S_SUSPENDED : S_READY; //puts the state in ready or suspended depending on autostart
        task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE
                - STACK_LR_OFFSET] = (uint32_t) task_body;  //assigns the current task_body
        task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE
                        - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT; //assigns default PSR value
        retval = task_list.nTasks;
        task_list.nTasks++;
	}
}

rtos_tick_t rtos_get_clock(void)
{
	return task_list.global_tick; //returns global clock
}

void rtos_delay(rtos_tick_t ticks)
{
    task_list.tasks[task_list.current_task].state = S_WAITING; //sets state to waiting
    task_list.tasks[task_list.current_task].local_tick = ticks; //makes a delay of the amount of ticks received
    dispatcher(kFromNormalExec); //calls dispatcher from current task
}

void rtos_suspend_task(void)
{
    task_list.tasks[task_list.current_task].state = S_SUSPENDED; //sets state to suspended
    dispatcher(kFromNormalExec); //calls dispatcher from current task
}

void rtos_activate_task(rtos_task_handle_t task)
{
    task_list.tasks[task].state = S_READY; //sets state to suspended
    dispatcher(kFromNormalExec); //calls dispatcher from current task
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type) //handles of scheduling, gets called from interruption or within task
{
    rtos_task_handle_t next_task = RTOS_INVALID_TASK; //sets next task as idle
    uint8_t index;
    int8_t highest = -1; //sets the current highest priority as -1
    for(index = 0 ; index < task_list.nTasks ; index ++) //for the number of current tasks
    {
        if ( highest < task_list.tasks[index].priority
                && (S_READY == task_list.tasks[index].state
                        || S_RUNNING == task_list.tasks[index].state)) // if the task is running and its priority is higher it's next
        {
            next_task = index;
            highest = task_list.tasks[index].priority;
        }
    }
    if(task_list.current_task != next_task) //if the current task is different from the next one
    {
        task_list.next_task = next_task; //sets that task as the next one
        context_switch(type); //calls for a context switch from within the task
    }

}

FORCE_INLINE static void context_switch(task_switch_type_e type) //TODO checar con el if first
{

    register uint32_t *sp asm("sp"); //pointer for current sp dir
    if(kFromISR == type) //if it comes from isr gives proper offset
    {
        task_list.tasks[task_list.current_task].sp = sp + SP_OFFSET;
    }
    else //it comes from normal execution and gives proper offset
    {
        task_list.tasks[task_list.current_task].sp = sp - SP_OFFSET;
    }
    task_list.current_task = task_list.next_task; //changes task
    task_list.tasks[task_list.current_task].state = S_RUNNING; //sets state to running
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; //uses pendSV to push the current stack

}

static void activate_waiting_tasks()
{
    uint32_t index = 0;
    for(index = 0 ; index < task_list.nTasks; index++) //goes through the array to all tasks
    {
        if(S_WAITING == task_list.tasks[index].state) //if they're on waiting
        {
            task_list.tasks[index].local_tick--; //reduces their local clock until 0
            if(0 == task_list.tasks[index].local_tick)
            {
                task_list.tasks[index].state = S_READY; //when the clock reaches 0 it's ready
            }
        }
    }
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
	{
//	    PRINTF("estamos en idle");
	}
}

/**********************************************************************************/
// ISR implementation
/**********************************************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	task_list.global_tick++;
	activate_waiting_tasks();
	dispatcher(kFromISR);
	reload_systick();
}

void PendSV_Handler(void)
{
    register uint32_t *r0 asm("r0");
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    r0 = task_list.tasks[task_list.current_task].sp;
    asm("mov r7,r0");
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_WritePinOutput(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else
	{
		count++;
	}
}
#endif
