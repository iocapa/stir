#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* Sys info */
/* Quartz - 16MHz */
/* PB2 - ADC in (Speed pot) */
/* PB1 - HALL in (Use analog comparator) */
/* PB0 - FAN_CTRL */

/* PID regulator parameters */
#define SYS_KP	(1.1f)
#define SYS_KI	(0.06f)
#define SYS_KD	(0.004f)
#define SYS_OFS	(0)
#define SYS_TS	(4) /* Hz */

/* System configuration */
#define NUM_DS_BITS 	(12) /* 2^value + 1 for max */
#define CAP_NUM_EVENTS	(2u) /* Length of queue for events filter (power of 2)*/
#define CAP_NUM_POLES 	(2u) /* Number of hall toggles per rotation */
#define SPEED_MIN		(200u) /* Minimum speed in RPM */
#define SPEED_MAX		(3000u) /* Maximum speed in RPM */

/* Pin configuration */
#define TACH_PIN	(1)
#define POT_PIN		(2)
#define OUT_PIN		(0)

/* Fixed scales */
#define ADC_MAX		(1023)
#define SCALE_FIX	(4096)
#define F_CAP		(2000000)
#define S_P_MIM		(60)

/* Internal macros */
#define TRUE		(1u)
#define FALSE		(0)

/* Fixed system macros */
#if ((F_CPU / 2048 / SYS_TS) < 256)
#define TSAMP_PRE	2048
#elif ((F_CPU / 4096 / SYS_TS) < 256)
#define TSAMP_PRE	4096
#elif ((F_CPU / 8192 / SYS_TS) < 256)
#define TSAMP_PRE	8192
#elif ((F_CPU / 16384 / SYS_TS) < 256)
#define TSAMP_PRE	16384
#else
#error "Could not solve for sample time"
#endif

/* Actual sample time */
#define SYS_TSAMP	(((float)F_CPU) / ((float)TSAMP_PRE) / (float)((int32_t)(F_CPU / TSAMP_PRE / SYS_TS)))

/* Internal values */
#define INT_GAIN	(SYS_TSAMP * SYS_KI * (float)SCALE_FIX)
#define DER_GAIN	((1.0f / SYS_TSAMP) * SYS_KD * (float)SCALE_FIX)
#define DS_MAX		(1u << NUM_DS_BITS)
#define INT_SAT		((int32_t)DS_MAX * SCALE_FIX)

/* Sigma delta modulator valriables */
typedef struct {
	/* Primary latch */
	uint16_t latch;
} ds_data_t;

/* PID internal variables */
typedef struct {
	/* Integrator accumulator */
	int32_t acc_int;

	/* Derrivator accumulator */
	int32_t acc_der;
} pid_data_t;

/* Capture data type */
typedef struct {
	/* Event queue */
	uint32_t ev_queue[CAP_NUM_EVENTS];

	/* Hight part of the 24 bit capture counter, low 8 bit in TIM0 */
	uint16_t high_val;

	/* Queue index */
	uint8_t index;

	/* Overflow flag */
	uint8_t ovf;
} capture_data_t;

/* Program data type */
typedef struct {
	/* Capture data */
	capture_data_t cap_data;

	/* PID data */
	pid_data_t pid_data;

	/* Delta sigma data */
	ds_data_t ds_data;

	/* Requested output value */
	uint16_t req_out;

	/* Conversion result */
	uint16_t adc_val;

	/* Synchronization flag */
	uint8_t volatile sync_req;
} program_data_t;

/* Static function prototypes */
static void configure_adc(void);
static void configure_gpio(void);
static void configure_sys_timer(void);
static void configure_cap_timer(void);
static void configure_comparator(void);
static void system_init(void);
static void add_in_cap_queue(uint32_t val);
static void compute_output(uint16_t val);
static void process_controller(void);
static uint16_t process_current_speed(void);
static uint16_t process_requested_setpoint(void);
static uint16_t process_main_pid(uint16_t sp, uint16_t fb);
static void handle_adc_irq(void);
static void handle_capture_irq(void);
static void handle_sample_timer_irq(void);
static void handle_capture_timer_irq(void);

/* Program data. Needs to be volatile. */
static volatile program_data_t prog_data;

/************************************************************************
 * @brief Configure the analog comparator
 */
static void configure_comparator(void) {
	/* Enable the bandgap */
	ACSR |= (1u << ACBG);

	/* Enable the IRQ */
	ACSR |= (1u << ACIE);
}

/************************************************************************
 * @brief Configure the capture timer
 */
static void configure_cap_timer(void) {
	/* Set the prescaler */
	/* We need a counter incrementing at about 2MHz */
	/* F_CPU / 2000000 = 8 */
	TCCR0B |= (1u << CS01);

	/* Enable the IRQ */
	TIMSK |= (1u << TOIE0);
}

/************************************************************************
 * @brief Configure the system timer
 */
static void configure_sys_timer(void) {
	/* Set compare and top */
	OCR1A = (uint8_t)(F_CPU / TSAMP_PRE / SYS_TS);
	OCR1C = (uint8_t)(F_CPU / TSAMP_PRE / SYS_TS);

	/* Enable interrupt */
	TIMSK |= (1u << OCIE1A);

	/* Write prescaler and start timer */
#if TSAMP_PRE == 2048
	TCCR1 |= ((1u << CTC1) | (1u << CS13) | (1u << CS12));
#elif TSAMP_PRE == 4096
	TCCR1 |= ((1u << CTC1) | (1u << CS13) | (1u << CS12) | (1u << CS10));
#elif TSAMP_PRE == 8192
	TCCR1 |= ((1u << CTC1) | (1u << CS13) | (1u << CS12) | (1u << CS11));
#elif TSAMP_PRE == 16384
	TCCR1 |= ((1u << CTC1) | (1u << CS13) | (1u << CS12) | (1u << CS11) | (1u << CS10));
#else
#error "No prescaler value"
#endif
}

/************************************************************************
 * @brief Configure the GPIO peripheral
 */
static void configure_gpio(void) {
	/* Input pins */
	DDRB &= ~((1u << TACH_PIN) | (1u << POT_PIN));

	/* Output pins */
	DDRB |= (1u << OUT_PIN);
}

/************************************************************************
 * @brief Configure the ADC peripheral
 */
static void configure_adc(void) {
	/* ADC 1 */
	ADMUX |= (1u);

	/* Disable digital buffers */
	DIDR0 |= ((1u << ADC1D) | (1 << AIN1D));

	/* Select prescaler = 128 */
	ADCSRA |= ((1u << ADPS2) | (1u << ADPS1) | (1u << ADPS0));

	/* Enable the IRQ */
	ADCSRA |= (1u << ADIE);

	/* Enable the peripheral */
	ADCSRA |= (1u << ADEN);

	/* Start a conversion */
	ADCSRA |= (1u << ADSC);
}

/************************************************************************
 * @brief Init the hardware peripherals
 */
static void system_init(void) {
	/* Configure GPIOs */
	configure_gpio();

	/* Configure the comparator */
	configure_comparator();

	/* ADC setup */
	configure_adc();

	/* Configure system timer */
	configure_sys_timer();

	/* Configure the capture timer */
	configure_cap_timer();

	/* Enable the interrupts */
	sei();
}

/************************************************************************
 * @brief Compute a new value for the sigma delta converter
 * @param val Requested value
 */
static void compute_output(uint16_t val) {
	/* Dac output */
	uint16_t dac;

	/* 1 bit dac result and pin*/
	if(prog_data.ds_data.latch & DS_MAX) {
		dac = DS_MAX;
		PORTB |= (1u << OUT_PIN);
	} else {
		dac = 0;
		PORTB &= ~(1u << OUT_PIN);
	}

	/* Compute the latch */
	prog_data.ds_data.latch = ((prog_data.ds_data.latch + val) - dac);
}

/************************************************************************
 * @brief Return the speed value
 */
static uint16_t process_current_speed(void) {
	/* For a value in RPM the formula is
	 * RPM = ((F_CAP * S_P_MIM * CAP_NUM_EVENTS) / CAP_NUM_POLES) / COUNT
	 * where ((F_CAP * S_P_MIM * CAP_NUM_EVENTS) / CAP_NUM_POLES) is constant
	 * F_CAP = TIMER frequency = 2000000Hz
	 * S_P_MIM = Seconds Per Minute
	 * COUNT = The average sum
	 */
	/* Holding the average for last CAP_NUM_EVENTS samples */
	uint32_t ave_samp;

	/* Return value with inital constant value */
	uint32_t rval;

	/* Local mean index */
	uint8_t index;

	/* Intial constant */
	rval = ((uint32_t)(F_CAP * S_P_MIM * CAP_NUM_EVENTS) / CAP_NUM_POLES);

	/* Known intial value */
	ave_samp = 0u;

	/* Disable interrupts */
	cli();

	/* Sum all the variables */
	for(index = 0u; index < CAP_NUM_EVENTS; index++) {
		ave_samp += prog_data.cap_data.ev_queue[index];
	}

	/* Enable interrupts back */
	sei();

	/* Compute the return */
	rval = rval / ave_samp;

	/* Return the speed */
	return (uint16_t)rval;
}

/************************************************************************
 * @brief Return the requested setpoint value
 */
static uint16_t process_requested_setpoint(void) {
	/* Requested speed value */
	uint32_t rspeed = ((((uint32_t)(SPEED_MAX - SPEED_MIN) + 1u) * SCALE_FIX) / ADC_MAX);

	/* Apply fixed point scaling factor */
	rspeed = rspeed * prog_data.adc_val;

	/* Scale and add to min */
	rspeed = (rspeed / SCALE_FIX) + SPEED_MIN;

	/* Return the scaled value */
	return (uint16_t)rspeed;
}

/************************************************************************
 * @brief Process the PID controller
 * @param sp Setpoint
 * @param fb Feedback
 * @return Output
 */
static uint16_t process_main_pid(uint16_t sp, uint16_t fb) {
	/* Error */
	int32_t err = ((int32_t)sp - (int32_t)fb);

	/* Output */
	int32_t out;

	/* Integrator value */
	int32_t int_val;

	/* Derrivator value */
	int32_t der_val, der_temp;

	/* Apply the proportional gain fix .10 format*/
	out = err * (int32_t)(SYS_KP * (float)SCALE_FIX);

	/* Apply the derrivative term */
	der_temp = (err * (int32_t)DER_GAIN);
	der_val = (der_temp - prog_data.pid_data.acc_der);
	prog_data.pid_data.acc_der = der_temp;
	out += der_val;

	/* Apply the integral term */
	int_val = (err * (int32_t)INT_GAIN); /* .10 */
	int_val = int_val + prog_data.pid_data.acc_int; /* .10 */
	out += int_val;

	/* Saturate integrator*/
	if(prog_data.pid_data.acc_int > INT_SAT) {
		prog_data.pid_data.acc_int = INT_SAT;
	} else if(prog_data.pid_data.acc_int < 0) {
		prog_data.pid_data.acc_int = 0;
	} else {
		prog_data.pid_data.acc_int = int_val;
	}

	/* Normalize the result */
	out = out / SCALE_FIX;

	/* Add the offset */
	out += SYS_OFS;

	/* Return the saturated output */
	if(out > (DS_MAX - 1)) {
		return (DS_MAX - 1);
	} else if(out < 0) {
		return 0;
	} else {
		return (uint16_t)out;
	}
}


/************************************************************************
 * @brief Run the main process function
 */
static void process_controller(void) {
	/* Current speed */
	uint16_t fb_speed = process_current_speed();

	/* Current setpoint */
	uint16_t sp_req = process_requested_setpoint();

	/* Update the requested output */
	prog_data.req_out = process_main_pid(sp_req, fb_speed);
}

/************************************************************************
 * @brief Add element to capture queue
 * @param val
 */
static void add_in_cap_queue(uint32_t val) {
	/* Temporary queue index */
	uint8_t q_index = (prog_data.cap_data.index & (CAP_NUM_EVENTS - 1));

	/* Add it in the queue */
	prog_data.cap_data.ev_queue[q_index] = val;

	/* Increment the index */
	prog_data.cap_data.index++;
}

/************************************************************************
 * @brief Handler for the ADC conversion complete interrupt
 */
static void handle_adc_irq(void) {
	/* Copy the value */
	prog_data.adc_val = ADC;

	/* Request a sync */
	prog_data.sync_req = TRUE;
}

/************************************************************************
 * @brief Handler for the hall capture interrupt
 */
static void handle_capture_irq(void){
	/* Temporary low byte */
	uint8_t lval;

	/* Temporary high value */
	uint16_t hval;

	/* Temporary timestamp */
	uint32_t tstamp;

	/* Get the low part */
	lval = TCNT0;

	/* het the high part */
	hval = prog_data.cap_data.high_val;

	/* Clear the current values */
	prog_data.cap_data.high_val = 0;
	TCNT0 = 0;

	/* Create the value */
	tstamp = ((uint32_t)hval << 8u) | lval;

	/* Add it in the queue */
	add_in_cap_queue(tstamp);
}

/************************************************************************
 * @brief Handler for the sample timer interrupt
 */
static void handle_sample_timer_irq(void) {
	/* Start a new conversion */
	ADCSRA |= (1u << ADSC);
}

/************************************************************************
 * @brief Handler for the capture timer interrupt
 */
static void handle_capture_timer_irq(void) {
	/* limit overflow */
	if(prog_data.cap_data.high_val < (2000000u / 256u)) {
		/* Just increment the variable */
		prog_data.cap_data.high_val++;
	} else {
		/* Add a max value to the queue */
		add_in_cap_queue(0xFFFFFFFFu);
	}

	/* Call the sd converter handler at 7812.5 Hz (2000000 / 256)*/
	compute_output(prog_data.req_out);
}

/************************************************************************
 * @brief ADC conversion complete IRQ
 */
ISR(ADC_vect) {
	/* Call the handler */
	handle_adc_irq();
}

/************************************************************************
 * @brief Timer 1 compare interrupt
 */
ISR(TIM1_COMPA_vect) {
	/* Call the handler */
	handle_sample_timer_irq();
}

/************************************************************************
 * @brief Analog comparator toggle interrupt
 */
ISR(ANA_COMP_vect) {
	/* Call the handler */
	handle_capture_irq();
}

/************************************************************************
 * @brief Timer 0 overflow vector
 */
ISR(TIM0_OVF_vect) {
	/* Call the handler */
	handle_capture_timer_irq();
}

/************************************************************************
 * @brief Main entry point
 */
void main(void) {
	/* Init the system */
	system_init();

	/* Main loop */
	while(TRUE) {
		/* Check for sync */
		if(prog_data.sync_req) {
			/*Run the process controller */
			process_controller();

			/* Reset the flag */
			prog_data.sync_req = FALSE;
		}
		/* TODO see sleep modes */
	}
}
