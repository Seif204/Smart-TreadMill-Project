#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PIR_SENSOR_PIN PD3
#define STOP_BUTTON_PIN PB5
#define RESUME_BUTTON_PIN PB4
#define RESET_TIME_PIN PB6

uint32_t stopwatch_seconds = 0;
uint8_t motor_speed = 50; // Initial motor speed
uint8_t treadmill_running = 0;
uint8_t treadmill_paused = 0;

void timer1_init() {
    TCCR1B |= (1 << WGM12)|(1 << CS11) | (1 << CS10); //  CTC mode & Enable Prescaler 64
    OCR1A = 15624; // compare value
    TIMSK |= (1 << OCIE1A); // Enable CTC interrupt 
}

ISR(TIMER1_COMPA_vect) {
    if (treadmill_running && !treadmill_paused) {
        stopwatch_seconds++;
    }
}


void timer0_pwm_init(uint8_t speed) {
    TCCR0 |= (1 << WGM00) | (1 << WGM01); // Fast PWM mode
    TCCR0 |= (1 << COM01) | (1 << CS00);  // Non-inverted PWM, no prescaler
    DDRB |= (1 << PB3); // Set PB3 as output 
    OCR0 = speed; // Set motor speed
}


void ext_interrupt_init() {
    GICR |= (1 << INT0) | (1 << INT1) | (1 << INT2); // Enable INT0, INT1, INT2
    MCUCR |= (1 << ISC01); // Rising edge of INT0
    MCUCR |= (1 << ISC10) | (1 << ISC11); // Rising edge of INT1
    MCUCSR &= ~(1 << ISC2); // Falling edge of INT2
}

ISR(INT0_vect) {
    if (motor_speed < 255) motor_speed += 50;
    timer0_pwm_init(motor_speed);
}

ISR(INT2_vect) {
    if (motor_speed >= 25) motor_speed -= 50;
    timer0_pwm_init(motor_speed);
}

ISR(INT1_vect) {
    if (PIND & (1 << PIR_SENSOR_PIN)) {
        if (treadmill_running) {
            stop_treadmill();
        } else {
            start_treadmill();
        }
    }
}


void update_display(uint32_t seconds) {
    uint8_t secs = seconds % 60; 
    uint8_t minutes = (seconds / 60) % 60; 
    uint8_t hours = seconds / 3600;

    // Display first digit in seconds
    PORTA = (1 << PA5);
    PORTC = secs % 10;
    _delay_ms(1);

    // Display second digit in seconds
    PORTA = (1 << PA4);
    PORTC = secs / 10;
    _delay_ms(1);

    // Display first digit in minutes
    PORTA = (1 << PA3);
    PORTC = minutes % 10;
    _delay_ms(1);

    // Display second digit in minutes
    PORTA = (1 << PA2);
    PORTC = minutes / 10;
    _delay_ms(1);

    // Display first digit in hours
    PORTA = (1 << PA1);
    PORTC = hours % 10;
    _delay_ms(1);

    // Display second digit in hours
    PORTA = (1 << PA0);
    PORTC = hours / 10;
    _delay_ms(1);
}


void start_treadmill() {
    treadmill_running = 1;
    treadmill_paused = 0;
    timer0_pwm_init(motor_speed);
}

void stop_treadmill() {
    treadmill_running = 0;
    timer0_pwm_init(0);
}

void pause_treadmill() {
    treadmill_paused = 1;
    timer0_pwm_init(0);
}

void resume_treadmill() {
    treadmill_paused = 0;
    if (treadmill_running) {
        timer0_pwm_init(motor_speed); 
    }
}
void reset_display() {
    stopwatch_seconds = 0; // Reset the stopwatch counter
    update_display(stopwatch_seconds); 
}

int main(void) {
  
    DDRB &= ~((1 << STOP_BUTTON_PIN) | (1 << RESUME_BUTTON_PIN)| (1 << RESET_TIME_PIN) ); // Set PB4, PB5 ,PB6 as input
    DDRC = 0x0F; // Set first 4 pins of PORTC as output 
    DDRA = 0x3F; // Set first 6 pins of PORTA as output 
    DDRD &= ~(1 << PIR_SENSOR_PIN); // Set PIR_SENSOR_PIN as input
    DDRB |= (1 << PB0) | (1 << PB1); // Set PB0 and PB1 as outputs
    PORTB |= (1 << PB0); // Set PB0 high to rotate the motor
    PORTB &= ~(1 << PB1); // Set PB1 low
    SREG |= (1 << 7);  // Enable global interrupts
    
 
    timer1_init();
    timer0_pwm_init(motor_speed);
    ext_interrupt_init();
    

    stop_treadmill(); // Initialize state Tradmaill

    while (1) {
        // Check if the stop button is pressed
        if (PINB & (1 << STOP_BUTTON_PIN)) {
            pause_treadmill(); 
            update_display(stopwatch_seconds); 
            while (PINB & (1 << STOP_BUTTON_PIN)); 
        }

        // Check if the resume button is pressed
        if (PINB & (1 << RESUME_BUTTON_PIN)) {
            resume_treadmill(); 
            while (PINB & (1 << RESUME_BUTTON_PIN));
        }
        
        // Reset the display
        if (PINB & (1 << RESET_TIME_PIN)) {
           reset_display();
           while (PINB & (1 << RESET_TIME_PIN)); // Wait for button release
}

        // Update the display if the treadmill is running and not paused
        if (treadmill_running && !treadmill_paused) {
            update_display(stopwatch_seconds);
        }

        _delay_ms(5);
    }

    return 0;
}
