#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

/// Keep the delay short
#define MAX_DELAY (F_CPU/1000UL)
/// Helper macro to return the maximum of two values
#define max(a,b) (a<b ? b : a)


// LUT of 32 ditlen values,
// (approx) linear in WPM
static uint16_t dit_len_lut[] = {
  120, 109, 100, 92, 86, 80, 75, 70,
   67,  63,  60, 57, 54, 52, 50, 48,
   46,  44,  43, 41, 40, 39, 37, 36,
   35,  34,  33, 32, 31, 30, 29, 28};

/// Initializes the touch interface
void init_touch() {
  // set PB0 as output
  DDRB  |= (1 << DDB0);
  // set 0
  PORTB &= ~(1 << DDB0);
  // set PB1 & PB2 as output (low Z)
  DDRB  |= ((1 << DDB1) | (1 << DDB2));
  // set 0 (GND), keeps paddles discharged
  PORTB &= ~((1 << DDB1) | (1 << DDB2));
}

/// Read touch paddles, returns 0 if none paddle is touched,
/// 0x01 if only the left is touched, 0x02 if only the right is
/// touched and 0x03 if both are touched.
uint8_t read_touch(uint32_t thres) {
  // config sense pins as input (high impedance)
  DDRB  &= ~((1<<DDB1) | (1<<DDB2));
  // start charging, set PB0 -> 1
  PORTB |= (1<<DDB0);
  // delay
  for (uint32_t i=0; i<thres; i++) { _NOP(); }
  // check values (if the pin is still 0, the paddle is touched).
  uint8_t value = ((PINB>>1) & 0x03) ^ 0x03;
  // Discharge...
  PORTB &= ~(1<<DDB0);  // set PB0 -> 0
  // ... set sense pins as output (low impedance)
  DDRB  |= ((1<<DDB1) | (1<<DDB2));
  // ... set sense pin to 0 (GND)
  PORTB &= ~( (1<<DDB1) || (1<<DDB2) );
  // wait for a short time to discharge completely
  _delay_us(10);
  // done.
  return value;
}

/// Calibrate the touch interface
uint32_t calibrate_touch() {
  uint32_t thres = 1;
  // Increment threshold until touch detects no value
  while (read_touch(thres) && (thres<MAX_DELAY)) {
    thres++;
  }
  // Increase threshold for reliable detection
  // (this decreses the sensitivity for the sake
  // of reduced false-positives).
  return 3*thres;
}


/// Configures the timer interrupt to be triggered every 1ms.
void init_timer() {
  // Set Timer on Compare Match (CTC) mode
  TCCR0A = 0x02;
  // clock source CLK/16
  TCCR0B = 0x03;
  // -> 1ms
  OCR0A  = 125-1;
  // enable interrupt (OC 0A)
  TIMSK  = (1<<OCIE0A);
}


/// Configures the output.
void init_key() {
  // set PB4 as output
  DDRB  |= (1 << DDB4);
  PORTB &= ~(1 << DDB4);
}

/// Sets the output.
void key(uint8_t on) {
  if (on) { PORTB |= (1<<DDB4); }
  else { PORTB &= ~(1<<DDB4); }
}

/// Toggles the output.
void toggle_key() {
  PORTB ^= (1<<DDB4);
}


/// Initializes the ADC.
void init_adc() {
  DDRB  |= (1 << DDB3);
  ADCSRA = (0 << ADEN) | (0 << ADSC) | (0 << ADATE) | (0 << ADIF) | (0 << ADIE) |
      (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADMUX = (0<<REFS2) | (0<<REFS1) | (0<<REFS0) | (0<<ADLAR) |
      (0<<MUX3) | (0<<MUX2) | (1<<MUX1) | (1<<MUX0);
  ADCSRA |= (1<<ADEN);
}

/// Starts a new conversion.
void start_adc() {
  ADCSRA |= (1<<ADSC);
}

/// Returns true if the ADC is ready.
uint8_t adc_available() {
  return (0 == (ADCSRA & (1<<ADSC)));
}

/// Returns the ADC value as a 5-bit (0,...,31) value.
uint8_t adc_value() {
  return (ADCW>>5);
}

/// Returns the dit length in ms as defined by the ADC value.
uint16_t get_dit_len() {
  /// @todo This formula for the dit-length is pretty shit: It is quiet sensitive
  /// at the interesting values around 20WPM.
  return dit_len_lut[adc_value()];
}


/** Possible states of the state machine. */
typedef enum {
  IDLE,         ///< Idle, wait for keys.
  SEND_DIT,     ///< Send a dit
  SEND_DAH,     ///< Send a dah
  SEND_DITDAH,  ///< Send a dit + dah
  SEND_DAHDIT   ///< Send a dah + dit
} State;

/// The current state.
volatile State state = IDLE;
/// The last state.
volatile State last_state = IDLE;
/// The current dit length in ms.
/// Gets updated on startup.
volatile uint16_t dit_len = 60;
/// Precomputed multiples of ditlen
volatile uint16_t dit_2len = 2*60;
volatile uint16_t dit_3len = 3*60;
volatile uint16_t dit_4len = 4*60;
volatile uint16_t dit_5len = 5*60;
volatile uint16_t dit_6len = 6*60;


int main(void)
{
  // Init timer interrupt
  init_timer();
  // Init touch interface
  init_touch();
  // Init output
  init_key();
  // Init ADC
  init_adc();

  key(1);
  uint32_t thres = 0;
  for (uint8_t i=0; i<100; i++) {
    thres = max(thres, calibrate_touch());
  }
  key(0);

  // on calibration error (delay threshold to large)
  while (MAX_DELAY == thres) {
    // -> blink trap
    toggle_key();
    _delay_ms(200);
  }

  // Measure speed
  start_adc();
  // Wait for the ADC to finish
  while (! adc_available()) { }
  // Compute dit lenght
  dit_len = get_dit_len();
  // Precompute multiples of dit len
  dit_2len = 2*dit_len; dit_3len = 3*dit_len; dit_4len = 4*dit_len;
  dit_5len = 5*dit_len; dit_6len = 6*dit_len;

  // Enable interrupts globally
  sei();

  // Go.
  state = IDLE;
  last_state = IDLE;

  // Poll paddles
  while(1) {
    // If waiting for the next symbol
    if (IDLE == state) {
      // Dispatch by paddle state
      // (left -> dit, right -> dah, both -> dit+dah or dah+dit)
      switch (read_touch(thres)) {
        // On left paddle
        case 0x01:
          last_state = IDLE;
          state = SEND_DIT;
          break;

        // On right paddle
        case 0x02:
          last_state = IDLE;
          state = SEND_DAH;
          break;

        // On both paddles
        case 0x03:
          if (SEND_DIT == last_state) {
            // If last state was a dit -> send dah + dit
            last_state = SEND_DIT;
            state = SEND_DAHDIT;
          } else {
            // If last state was a dah -> send dit + dah
            last_state = state;
            state = SEND_DITDAH;
          }
          break;

        // Otherwise stay in idle
        default:
          break;
      }

      // If still in idle -> check speed dial and update dit length
      if ((IDLE == state) && adc_available()) {
        // If ADC is ready, update dit len
        dit_len = get_dit_len();
        // Precompute multiples of dit len
        dit_2len = 2*dit_len; dit_3len = 3*dit_len; dit_4len = 4*dit_len;
        dit_5len = 5*dit_len; dit_6len = 6*dit_len;
        // and restart measurement
        start_adc();
      }
    }
  }
}


ISR (TIMER0_COMPA_vect)
{
  static uint16_t count = 0;

  if (SEND_DIT == state) {
    // Send a dit (10)
    if (0 == count) {
      key(1); count++;
    } else if (dit_len == count) {
      key(0); count++;
    } else if (dit_2len == count) {
      last_state = SEND_DIT;
      state = IDLE;
      count = 0;
    } else {
      count++;
    }
  } else if (SEND_DAH == state) {
    // Send a dah (1110)
    if (0 == count) {
      key(1); count++;
    } else if (dit_3len == count) {
      key(0); count++;
    } else if (dit_4len == count) {
      last_state = SEND_DAH;
      state = IDLE;
      count = 0;
    } else {
      count++;
    }
  } else if (SEND_DITDAH == state) {
    // Send a dit+dah (101110)
    if (0 == count) {
      key(1); count++;
    } else if (dit_len == count) {
      key(0); count++;
    } else if (dit_2len == count) {
      key(1); count++;
    } else if (dit_5len == count) {
      key(0); count++;
    } else if (dit_6len == count) {
      last_state = SEND_DITDAH;
      state = IDLE;
      count = 0;
    } else {
      count++;
    }
  } else if (SEND_DAHDIT == state) {
    // Send a dah+dit (111010)
    if (0 == count) {
      key(1); count++;
    } else if (dit_3len == count) {
      key(0); count++;
    } else if (dit_4len == count) {
      key(1); count++;
    } else if (dit_5len == count) {
      key(0); count++;
    } else if (dit_6len == count) {
      last_state = SEND_DAHDIT;
      state = IDLE;
      count = 0;
    } else {
      count++;
    }
  }
}
