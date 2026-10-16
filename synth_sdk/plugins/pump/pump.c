/*
 * Tempo-synced volume pump / gain shaper.
 *
 * This is not an audio-triggered sidechain compressor: it does not listen for
 * a kick. It uses the OP-1 beat clock to duck whatever audio reaches this FX.
 * At rate 1, the gain envelope restarts on every beat (four pumps per 4/4 bar).
 * Higher rates use shorter subdivisions of that beat.
 *
 * Knobs (blue, green, white, red):
 *   blue  - depth: 0 = no ducking, max = silence at the start of a pump
 *   green - rate: 1, 2, 4, 8, or 16 subdivisions per beat
 *   white - hold: how much of each subdivision stays at the ducked level (0..1/2)
 *   red   - decay: how much of the subdivision is used to recover to full level
 *
 * The envelope has three stages in each subdivision: hold low, recover
 * linearly, then remain at full level until the next subdivision. All phase
 * and gain values use integer fixed-point math because this runs in the audio
 * interrupt and the OP-1 has no usable floating-point unit.
 * Copyright (C) <year> <you>. GPL-3.0-or-later with the OP-1 firmware
 * permission (see LICENSE and NOTICE).
 */
#include "op1_plugin.h"

/* 1/16384-sample clock units: 64 output samples at normal speed. */
#define PUMP_CLOCK_UNITS_PER_BLOCK 0x100000
#define PUMP_DEFAULT_BEAT_LEN      (22050 << 14) /* 120 BPM if no clock exists yet. */
#define PUMP_MIN_BEAT_LEN          (256 << 14)   /* Same minimum clock length used by the synced room FX. */
#define PUMP_MAX_CLOCK_STEP        0x400000     /* Reject implausible clock jumps; fall back to tempo. */

typedef struct {
  /* Countdown to the next beat while the OP-1 clock is stopped/unavailable.
   * The real clock is used whenever it is moving. */
  int32_t free_running_next_beat;
} pump_state_t;

/* FX positions can run independently: 0 synth, 1 drum, 2 master. */
static pump_state_t pump_states[3];

static void pump_init(int position) {
  pump_states[position % 3].free_running_next_beat = -1;
}

/* Turn the green knob into common musical subdivisions per beat:
 * 1 = quarter notes, 2 = eighths, 4 = sixteenths, 8 = thirty-seconds,
 * 16 = sixty-fourths. Five equal knob zones select the five rates. */
static int pump_subdivisions(int32_t rate_knob) {
  int selection = (rate_knob * 5) >> 15; /* 0..4 */
  if (selection > 4) selection = 4;
  return 1 << selection;
}

/* Calculate the same three-stage gain curve used by the audio callback. */
static inline int32_t pump_gain_at_phase(int32_t phase, int32_t depth,
                                         int32_t hold, int32_t decay,
                                         int32_t recovery_scale) {
  int32_t ducked_gain = 32767 - depth;
  if (phase < hold) return ducked_gain;
  if (decay > 0 && phase < hold + decay) {
    int32_t recovery = ((phase - hold) * recovery_scale) >> 15;
    return ducked_gain + ((depth * recovery) >> 15);
  }
  return 32767;
}

/* Apply a gain envelope to one 64-sample mono block, in place. */
static void pump_render(int position, int16_t *audio, const int16_t *params) {
  pump_state_t *state = &pump_states[position % 3];

  /* Convert the firmware's raw knob words to normalized 0..32767 values. */
  int32_t depth = op1_fx_param_q15(params, 0);
  int32_t rate_knob = op1_fx_param_q15(params, 1);
  int32_t hold_knob = op1_fx_param_q15(params, 2);
  int32_t decay_knob = op1_fx_param_q15(params, 3);

  int subdivisions = pump_subdivisions(rate_knob);

  /* Read the shared clock once per render block, as required by the API. */
  op1_beat_t clock;
  op1_beat_clock(&clock);

  int32_t beat_len = clock.len;
  if (beat_len < PUMP_MIN_BEAT_LEN) {
    /* Use a sane 120 BPM fallback until the firmware clock is initialized. */
    beat_len = PUMP_DEFAULT_BEAT_LEN;
  }

  /* A valid moving clock follows tempo and, when enabled, tape speed. The
   * clock countdown and block step use the same 1/16384-sample units. */
  int clock_is_moving = clock.len >= PUMP_MIN_BEAT_LEN &&
                        clock.step > 0 && clock.step <= PUMP_MAX_CLOCK_STEP &&
                        clock.next >= 0 && clock.next <= clock.len;
  int32_t time_to_next_beat;
  int32_t block_step;

  if (clock_is_moving) {
    time_to_next_beat = clock.next;
    block_step = clock.step;
    /* Preserve its phase in case tape-following stops on the next block. */
    state->free_running_next_beat = clock.next;
  } else {
    /* When the OP-1 clock follows a stopped tape, it stands still. Keep a
     * private beat countdown moving at the current tempo so the pump continues. */
    if (state->free_running_next_beat < 0 ||
        state->free_running_next_beat > beat_len) {
      state->free_running_next_beat = beat_len;
    }
    time_to_next_beat = state->free_running_next_beat;
    block_step = PUMP_CLOCK_UNITS_PER_BLOCK;

    state->free_running_next_beat -= block_step;
    if (state->free_running_next_beat < 0) {
      state->free_running_next_beat += beat_len;
    }
  }

  /* `next` counts down to the beat, so beat_len - next is elapsed beat time.
   * Convert that to a Q15 cycle phase. The 64-bit work and division happen
   * once per block, never inside the per-sample loop. Masking keeps only the
   * position within the current subdivision. */
  int64_t elapsed = (int64_t)beat_len - time_to_next_beat;
  int64_t phase_numerator = elapsed * subdivisions * 32768;
  int32_t phase = (int32_t)(phase_numerator / beat_len) & 32767;

  /* Convert clock movement per block to movement per audio sample, then to
   * Q15 subdivision phase. The clock step is a multiple of 64 in this API. */
  int32_t clock_step_per_sample = block_step >> 6;
  int64_t phase_step_numerator = (int64_t)clock_step_per_sample * subdivisions * 32768;
  int32_t phase_step = (int32_t)(phase_step_numerator / beat_len);

  /* Hold is capped at half of a subdivision. Clamp decay to the remaining
   * portion, so hold + decay can never extend beyond the next pump trigger. */
  int32_t hold = hold_knob >> 1;
  int32_t max_decay = 32767 - hold;
  int32_t decay = (decay_knob * max_decay) >> 15;
  int32_t recovery_scale = decay > 0 ? (32767 << 15) / decay : 0;
  for (int sample = 0; sample < OP1_FX_BLOCK; sample++) {
    /* Hold low, ramp back up, then stay full until the next subdivision. */
    int32_t gain = pump_gain_at_phase(phase, depth, hold, decay, recovery_scale);

    /* Multiply in Q15 and saturate before writing the processed sample back. */
    audio[sample] = op1_sat16((audio[sample] * gain) >> 15);

    /* Advance phase without per-sample division or modulo. Phase wraps once
     * it passes the end of the Q15 subdivision cycle. */
    phase += phase_step;
    if (phase >= 32768) phase -= 32768;
  }
}

/* ------------------------------- screen -------------------------------- */
/* The SVG supplies the static frame and grid. The GUI callback draws the
 * changing envelope and playhead on top of it. */
#define PUMP_GRAPH_X       24
#define PUMP_GRAPH_Y       32
#define PUMP_GRAPH_W       272
#define PUMP_GRAPH_H       80
#define PUMP_GRAPH_SEGMENTS 48
#define PUMP_METER_Y       145

static uint32_t pump_screen_beat_phase;

static const uint8_t pump_col_envelope[4] = {0x00, 0xED, 0x95, 0xFF};
static const uint8_t pump_col_cursor[4] = {0xFF, 0x3A, 0x5D, 0xFF};
static const uint8_t pump_col_knobs[4][4] = {
    {0x4D, 0x9E, 0xFF, 0xFF}, /* depth / blue */
    {0x00, 0xED, 0x95, 0xFF}, /* rate / green */
    {0xFF, 0xFF, 0xFF, 0xFF}, /* hold / white */
    {0xFF, 0x3A, 0x5D, 0xFF}  /* decay / red */
};

static void pump_screen_init(void *screen) {
  (void)screen;
  pump_screen_beat_phase = 0;
}

static void pump_screen_update(void *screen, int dt) {
  int16_t knobs[4];
  op1_fx_knobs_live(knobs);

  int32_t depth = knobs[0] < 0 ? 0 : knobs[0];
  int32_t rate_knob = knobs[1] < 0 ? 0 : knobs[1];
  int32_t hold_knob = knobs[2] < 0 ? 0 : knobs[2];
  int32_t decay_knob = knobs[3] < 0 ? 0 : knobs[3];
  int subdivisions = pump_subdivisions(rate_knob);
  int32_t hold = hold_knob >> 1;
  int32_t decay = (decay_knob * (32767 - hold)) >> 15;
  int32_t recovery_scale = decay > 0 ? (32767 << 15) / decay : 0;

  /* Mirror the real clock when it is moving. If it is stopped, advance a
   * display-only phase from dt, using the clock's tempo (or 120 BPM fallback). */
  op1_beat_t clock;
  op1_beat_clock(&clock);
  int32_t beat_len = clock.len >= PUMP_MIN_BEAT_LEN ? clock.len : PUMP_DEFAULT_BEAT_LEN;
  int clock_is_moving = clock.len >= PUMP_MIN_BEAT_LEN &&
                        clock.step > 0 && clock.step <= PUMP_MAX_CLOCK_STEP &&
                        clock.next >= 0 && clock.next <= clock.len;
  if (clock_is_moving) {
    int64_t elapsed = (int64_t)clock.len - clock.next;
    pump_screen_beat_phase = (uint32_t)((elapsed * 32768) / clock.len) & 32767;
  } else {
    int32_t beat_ms = (int32_t)(((int64_t)beat_len * 1000) / (44100LL << 14));
    if (beat_ms < 1) beat_ms = 1;
    int32_t frame_dt = dt < 0 ? 0 : (dt > 250 ? 250 : dt);
    /* Ignore long UI pauses rather than making the marker jump. */
    pump_screen_beat_phase += (uint32_t)(((int64_t)frame_dt * 32768) / beat_ms);
    if (pump_screen_beat_phase >= 32768) pump_screen_beat_phase -= 32768;
  }

  op1_screen_draw(screen, dt);
  void *raster = op1_raster();
  op1_pt previous = {0, 0};

  /* Plot one full beat of the gain curve. Repeating the envelope makes the
   * selected subdivisions visible; the green line falls when audio is ducked. */
  for (int point = 0; point <= PUMP_GRAPH_SEGMENTS; point++) {
    int32_t beat_phase = (point * 32768) / PUMP_GRAPH_SEGMENTS;
    int32_t cycle_phase = (beat_phase * subdivisions) & 32767;
    int32_t gain = pump_gain_at_phase(cycle_phase, depth, hold, decay, recovery_scale);
    op1_pt current = {
        OP1_Q10(PUMP_GRAPH_X) + (OP1_Q10(PUMP_GRAPH_W) * point) / PUMP_GRAPH_SEGMENTS,
        OP1_Q10(PUMP_GRAPH_Y) +
            (int32_t)(((int64_t)OP1_Q10(PUMP_GRAPH_H) * (32767 - gain)) / 32767)};
    if (point > 0) op1_raster_line(raster, &previous, &current, 0x600, pump_col_envelope);
    previous = current;
  }

  /* Mark each pump trigger within the beat. */
  for (int tick = 0; tick < subdivisions; tick++) {
    int32_t x = OP1_Q10(PUMP_GRAPH_X) + (OP1_Q10(PUMP_GRAPH_W) * tick) / subdivisions;
    op1_pt top = {x, OP1_Q10(PUMP_GRAPH_Y - 3)};
    op1_pt bottom = {x, OP1_Q10(PUMP_GRAPH_Y + PUMP_GRAPH_H + 3)};
    op1_raster_line(raster, &top, &bottom, 0x200, pump_col_knobs[1]);
  }

  /* Move the red playhead across the beat, aligned to the OP-1 clock. */
  int32_t cursor_x = OP1_Q10(PUMP_GRAPH_X) +
                     (int32_t)(((int64_t)OP1_Q10(PUMP_GRAPH_W) * pump_screen_beat_phase) >> 15);
  op1_pt cursor_top = {cursor_x, OP1_Q10(PUMP_GRAPH_Y - 5)};
  op1_pt cursor_bottom = {cursor_x, OP1_Q10(PUMP_GRAPH_Y + PUMP_GRAPH_H + 5)};
  op1_raster_line(raster, &cursor_top, &cursor_bottom, 0x600, pump_col_cursor);

  /* Four small meters echo the live knob positions using OP-1 knob colours. */
  for (int knob = 0; knob < 4; knob++) {
    int32_t left = OP1_Q10(PUMP_GRAPH_X + knob * 68);
    int32_t width = (OP1_Q10(60) * (knobs[knob] < 0 ? 0 : knobs[knob])) >> 15;
    op1_pt a = {left, OP1_Q10(PUMP_METER_Y)};
    op1_pt b = {left + width, OP1_Q10(PUMP_METER_Y)};
    if (width > 0) op1_raster_line(raster, &a, &b, 0x800, pump_col_knobs[knob]);
  }
}

/* Register the audio and screen callbacks. No knob encoder presses are used. */
const op1_fx_t FX_pump = {
    pump_init,
    pump_render,
    {OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE}};

const op1_screen_t SCREEN_pump = {pump_screen_init, pump_screen_update};
