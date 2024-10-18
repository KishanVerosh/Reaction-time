#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define SSD_P PORTD //define SSD port
#define SSD_D DDRD //define SSD data direction register
#define SSD_DIG_P PORTC //define SSD digit port
#define SSD_DIG_D DDRC  //define SSD data direction register
#define SSD_DIG0 3 //define SSD digit 3 pin
#define SSD_DIG1 2 //define SSD digit 2 pin
#define SSD_DIG2 1 //define SSD digit 1 pin
#define SSD_DIG3 0 //define SSD digit 0 pin
#define START_LED 5 //define Start LED pin
#define STOP_LED 4 //define Stop LED pin

void init(void);
void initSsd(void);
void displaySsd(uint16_t val,uint16_t round);
void decodeSsd(uint16_t val);
void initADC();
uint16_t ADC_read();
void timer0_init();

uint8_t ssdVal[] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};
uint8_t digival[] = {0, 0, 0, 0};

volatile uint16_t start_time, end_time;
volatile uint16_t time;
volatile char test=0;
volatile uint16_t delay_period = 0;
volatile uint16_t timer0_millis = 0;
volatile uint8_t pRs=0;

int main(void) {
    init();
    initADC();
    timer0_init();
    sei(); // Enable global interrupts
    
    while(1){
        displaySsd(time,500);
    }
    return 0;
}
ISR(TIMER0_COMPA_vect) {
    timer0_millis++;  // Increment the millisecond counter
    if (timer0_millis == delay_period) {
    	PORTC |= 1 << STOP_LED;
       }
    }

ISR(PCINT0_vect) {
    if (!(PINB & (1 << PB0)) && pRs==0) {
    	timer0_millis = 0;
    	delay_period = ADC_read()*4; // read random number
        start_time = timer0_millis;
        PORTC |= 1 << START_LED; // start LED turn on
        pRs = 1;
    } else if (!(PINB & (1 << PB1))&& pRs==1) {
        end_time = timer0_millis;
        time = end_time - start_time - delay_period;
        pRs = 2;
    } else if(!(PINB & (1 << PB0)) && pRs==2){ // reset the systerm
    	PORTC &= ~(1 << START_LED | 1 << STOP_LED);
    	time = 0;
    	pRs = 0;
    	_delay_ms(500);
    }
}

void init(){
    SSD_D = 0xff; // Set SSD port as output
    initSsd(); // Initialize SSD pins
    
    DDRB &= ~(1 << PB0 | 1 << PB1); // Set button pins as input
    PORTB |= (1 << PB0 | 1 << PB1);
    PCICR |= (1 << PCIE0); // Enable pin change interrupt for PCINT
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1); 
}

void initSsd(void){
    SSD_DIG_D |= (1<<SSD_DIG0) | (1<<SSD_DIG1) | (1<<SSD_DIG2) | (1<<SSD_DIG3) | (1 << START_LED) | (1 << STOP_LED);
}

void displaySsd(uint16_t val,uint16_t round){
    decodeSsd(val);
    uint16_t i = 0;
    for(i = 0; i < round; i++){
    	// Display digit 1
        SSD_P = ssdVal[digival[0]];
        SSD_DIG_P |= (1<<SSD_DIG0);
        _delay_us(100);
        SSD_DIG_P &= ~(1<<SSD_DIG0);
        SSD_P = 0x00;

	// Display digit 2
        SSD_P = ssdVal[digival[1]];
        SSD_DIG_P |= (1<<SSD_DIG1);
        _delay_us(100);
        SSD_DIG_P &= ~(1<<SSD_DIG1);
        SSD_P = 0x00;

	// Display digit 3
        SSD_P = ssdVal[digival[2]];
        SSD_DIG_P |= (1<<SSD_DIG2);
        _delay_us(100);
        SSD_DIG_P &= ~(1<<SSD_DIG2);
        SSD_P = 0x00;

	// Display digit 4
        SSD_P = ssdVal[digival[3]];
        SSD_DIG_P |= (1<<SSD_DIG3);
        _delay_us(100);
        SSD_DIG_P &= ~(1<<SSD_DIG3);
        SSD_P = 0x00;
    }
}

void decodeSsd(uint16_t val){
    digival[0] = val % 10;
    digival[1] = (val / 10) % 10; 
    digival[2] = (val / 100) % 10; 
    digival[3] = (val / 1000) % 10; 
}

void initADC() {
    // Set the reference voltage to AVCC with external capacitor at AREF pin
    ADMUX |= (1 << REFS0); // REFS1 = 0, REFS0 = 1
    ADMUX &= (1 << REFS1);
    
    ADMUX &= ~(0x1F); // Clear the previous channel selection

    ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // Set the prescaler to 8
    ADCSRA &= ~(1 << ADPS2);

    ADCSRA |= (1 << ADEN); // Enable the ADC
}

uint16_t ADC_read() {
    // Start the ADC conversion
    ADCSRA |= (1 << ADSC);
    
    while (ADCSRA & (1 << ADSC)); // Wait for the conversion to complete
    return ADC;
}

void timer0_init() {
    TCCR0A |= (1 << WGM01); // Set CTC mode
    TCCR0A &= ~(1<<WGM02 | 1<<WGM00);
    
    TCCR0B |= 1<<CS01; // Set the prescaler to 8
    TCCR0B &= ~(1<<CS02 | 1<<CS00);
    
    TIMSK0 |= (1 << OCIE0A); // Enable Timer0 compare interrupt
    OCR0A = 124; // Set compare value for 1ms interrupt
    
}

