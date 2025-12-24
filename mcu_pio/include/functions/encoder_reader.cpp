// Hall-effect encoder reader with interrupt-based tick counting
// Plug-and-play: just modify pin numbers in caterpillar_pins.h

#pragma once
#include "pinmap/caterpillar_pins.h"

// ============ Encoder Tick Counters ============
// Volatile because modified in ISRs
volatile int32_t enc_fl_ticks = 0;
volatile int32_t enc_fr_ticks = 0;
volatile int32_t enc_bl_ticks = 0;
volatile int32_t enc_br_ticks = 0;

// ============ Interrupt Service Routines ============
// IRAM_ATTR ensures ISR code is in RAM for faster execution

void IRAM_ATTR enc_fl_isr() { enc_fl_ticks++; }

void IRAM_ATTR enc_fr_isr() { enc_fr_ticks++; }

void IRAM_ATTR enc_bl_isr() { enc_bl_ticks++; }

void IRAM_ATTR enc_br_isr() { enc_br_ticks++; }

// ============ Setup Function ============
void setupEncoders() {
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);

  attachInterrupt(ENC_FL_PIN, enc_fl_isr, RISING);
  attachInterrupt(ENC_FR_PIN, enc_fr_isr, RISING);
  attachInterrupt(ENC_BL_PIN, enc_bl_isr, RISING);
  attachInterrupt(ENC_BR_PIN, enc_br_isr, RISING);
}

// ============ Get Current Ticks ============
EncoderRaw getEncoderTicks() {
  EncoderRaw enc;
  // Disable interrupts briefly to get consistent reading
  noInterrupts();
  enc.fl_ticks = enc_fl_ticks;
  enc.fr_ticks = enc_fr_ticks;
  enc.bl_ticks = enc_bl_ticks;
  enc.br_ticks = enc_br_ticks;
  interrupts();
  return enc;
}

// ============ Reset Ticks (optional) ============
void resetEncoderTicks() {
  noInterrupts();
  enc_fl_ticks = 0;
  enc_fr_ticks = 0;
  enc_bl_ticks = 0;
  enc_br_ticks = 0;
  interrupts();
}
