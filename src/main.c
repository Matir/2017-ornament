#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr_mcu_section.h"

// Settings
#define DESIRED_FRAMERATE 60
#define NUM_LEDS 6
#define PWM_FRAME_MASK 0x7f
#define MAX_BRIGHTNESS_MASK PWM_FRAME_MASK
#define TIMER_INTERVAL F_CPU/(DESIRED_FRAMERATE*(PWM_FRAME_MASK+1))

const uint8_t MAX_BRIGHTNESS[NUM_LEDS] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
const uint8_t RAMP_UP_TIME[NUM_LEDS] =   {0x7F, 0x4F, 0x3F, 0x1F, 0x04, 0x12};
const uint8_t RAMP_DOWN_TIME[NUM_LEDS] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// INTERVAL is 4x as long as ramp up/down time
const uint8_t INTERVAL[NUM_LEDS] =       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Current brightness
uint8_t brightness[NUM_LEDS]     = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Current frame
uint16_t px_frame_no[NUM_LEDS]   = {0};

// Update frame
volatile uint8_t update_frame = 1;
volatile uint32_t frame_id = 0;
volatile uint8_t running_brightness = 0;

// Declarations
void setup(void);
void update_loop_step();
void pwm_frame_update(uint8_t frame_no);
void calc_brightness(uint8_t which);

// Handle the Timer compare
ISR(TIMER0_COMPA_vect) {
  update_frame = 1;
}

void setup(void) {
  // Disable interrupts while setting up
  cli();

  // Low 6 pins are output
  DDRD = 0x3f;

  // Enable timer for next frame
  OCR0A = TIMER_INTERVAL;   // Interval
  TIMSK = 0x01;             // Interrupt on compare
  TCCR0A = 0x02;            // CTC Mode
  TCCR0B = 0x01;            // No prescaler

  // Re-enable interrupts
  sei();
}

__attribute__((noreturn))
void main(void) {
  setup();
  while (1) {
    // TODO: sleep?
    if (update_frame) {
      cli();
      update_loop_step();
      frame_id++;
      update_frame = 0;
      sei();
    }
  }
}

void update_loop_step() {
  static uint8_t pwm_frame = 0;
  pwm_frame &= PWM_FRAME_MASK;
  if (!pwm_frame) {
    for(uint8_t i=0; i<NUM_LEDS; i++)
      calc_brightness(i);
  }
  pwm_frame_update(pwm_frame++);
}

void calc_brightness(uint8_t which) {
  running_brightness = 1;
  uint32_t brightness_tmp;
  uint16_t max_frame = (
      RAMP_UP_TIME[which] +
      RAMP_DOWN_TIME[which] +
      INTERVAL[which] * 4);
  uint16_t cur_frame = px_frame_no[which] + 1;
  if (cur_frame > max_frame)
    cur_frame = 0;
  px_frame_no[which] = cur_frame;

  if (cur_frame < RAMP_UP_TIME[which]) {
    brightness_tmp = MAX_BRIGHTNESS[which];
    brightness_tmp *= cur_frame;
    brightness_tmp /= RAMP_UP_TIME[which];
    brightness[which] = (uint8_t)(brightness_tmp & MAX_BRIGHTNESS_MASK);
  } else {
    cur_frame -= RAMP_UP_TIME[which];
    if (cur_frame < RAMP_DOWN_TIME[which]) {
      brightness_tmp = MAX_BRIGHTNESS[which];
      brightness_tmp *= (RAMP_DOWN_TIME[which] - cur_frame);
      brightness_tmp /= RAMP_DOWN_TIME[which];
      brightness[which] = (uint8_t)(brightness_tmp & MAX_BRIGHTNESS_MASK);
    } else {
      // Waiting INTERVAL
      brightness[which] = 0;
    }
  }
  running_brightness = 0;
}

void pwm_frame_update(uint8_t frame_no) {
  uint8_t out = 0;
  for(uint8_t i=0; i<NUM_LEDS; i++) {
    if(brightness[i] < frame_no)
      out |= (1<<i);
  }
  PORTD = out;
}

// simavr stuff
const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ = {
  {AVR_MCU_VCD_SYMBOL("update_frame"), .what = &update_frame},
  {AVR_MCU_VCD_SYMBOL("frame_id"), .what = &frame_id},
  {AVR_MCU_VCD_SYMBOL("running_bright"), .what = &running_brightness},
};
