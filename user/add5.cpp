/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "userosc.h"
#include "LCWPitchTable.h"
#include "LCWAntiAliasingTable.h"
#include "LCWOscWaveSource.h"
#include "LCWClipCurveTable.h"

#define LCW_OSC_TIMER_BITS (LCW_PITCH_DELTA_VALUE_BITS)
#define LCW_OSC_TIMER_MAX (1 << LCW_OSC_TIMER_BITS)
#define LCW_OSC_TIMER_MASK ((LCW_OSC_TIMER_MAX) - 1)

#define LCW_OSC_FRAC_BITS (LCW_OSC_TIMER_BITS - LCW_OSC_TABLE_BITS)
#define LCW_OSC_FRAC_MAX (1 << LCW_OSC_FRAC_BITS)
#define LCW_OSC_FRAC_MASK ((LCW_OSC_FRAC_MAX) - 1)

static struct {
  float shape = 0;
  float shiftshape = 0;
  uint32_t wave = 0;    // [0 .. 2]
} s_param;

static struct {
  uint32_t timer1 = 0;
  uint32_t timer2 = 0;
  int32_t pitch1 = 0;   // s7.24
  int32_t pitch2 = 0;   // s7.24
  int32_t shape_lfo = 0;
} s_state;

static const LCWOscWaveSource *s_waveSources[] = {
  &gLcwOscTriSource,
  &gLcwOscPulseSource,
  &gLcwOscSawSource
};

// return s15.16
__fast_inline int32_t myLookUp(
  int32_t t, int32_t dt, const LCWOscWaveSource *src)
{
    // 桁溢れ対策として、q16に変換
    uint32_t dt0 = dt >> (LCW_OSC_TIMER_BITS - 16);

    int32_t tmp[] = { 0, 0 };
    for (int32_t i=0; i<src->count; i++) {
      const int16_t *p = src->tables[i];
  
      // AAテーブルはdt=0.5までしか定義されていない
      uint32_t j = (dt0 * src->factors[i]) >> (16 - (LCW_AA_TABLE_BITS + 1));
      if ( (uint32_t)LCW_AA_TABLE_SIZE <= j ) {
        break;
      }

      int32_t gain = gLcwAntiAiliasingTable[j];
      uint32_t t0 = t >> LCW_OSC_FRAC_BITS;
      uint32_t t1 = (t0 + 1) & LCW_OSC_TABLE_MASK;

      // s15.16
      tmp[0] += ( (p[t0] * gain) >> (14 + LCW_AA_VALUE_BITS - 16) );
      tmp[1] += ( (p[t1] * gain) >> (14 + LCW_AA_VALUE_BITS - 16) );
    }

    // 端数14bitで補間
    const int32_t diff = tmp[1] - tmp[0];
    int32_t delta = (t & LCW_OSC_FRAC_MASK) >> (LCW_OSC_FRAC_BITS - 14);
    return tmp[0] + ((diff * delta) >> 14);
}

// in/return : s7.24
__fast_inline int32_t lookupClipCurve(int32_t x)
{
  const int32_t x2 = ( x < 0 ) ? -x : x;
  const int32_t i = x2 >> (LCW_CLIP_CURVE_VALUE_BITS - LCW_CLIP_CURVE_FRAC_BITS);

  int32_t ret;
  if ( i < (LCW_CLIP_CURVE_TABLE_SIZE - 1) ) {
    const uint32_t frac = (uint32_t)x2 & (0x00FFFFFF >> LCW_CLIP_CURVE_VALUE_BITS);
    const int32_t val = gLcwClipCurveTable[i];
    const int32_t diff = gLcwClipCurveTable[i+1] - val;
    const int64_t tmp = (int64_t)frac * diff;

    ret = val + (tmp >> (LCW_CLIP_CURVE_VALUE_BITS - LCW_CLIP_CURVE_FRAC_BITS));
  }
  else {
    ret = gLcwClipCurveTable[LCW_CLIP_CURVE_TABLE_SIZE - 1];
  }

  return ( x < 0 ) ? -ret : ret;
}

void OSC_INIT(uint32_t platform, uint32_t api)
{
  s_param.shape = 0.f;
  s_param.shiftshape = 0.f;
  s_param.wave = 0;

  s_state.timer1 = 0;
  s_state.timer2 = 0;
  s_state.pitch1 =
  s_state.pitch2 = (LCW_NOTE_NO_A4 << 24) / 12;
  s_state.shape_lfo = 0;
}

void OSC_CYCLE(const user_osc_param_t * const params,
               int32_t *yn,
               const uint32_t frames)
{
  // s11.20に拡張してから、整数部がoctaveになるように加工
  int32_t pitch1 = (int32_t)params->pitch << 12;
  pitch1 = (pitch1 - (LCW_NOTE_NO_A4 << 20)) / 12;
  const int32_t note = (int32_t)si_roundf(24.f * s_param.shape);
  // [0 .. 24] -> [-12 .. +12]
  int32_t pitch2 = pitch1 + (((note - 12) << 20) / 12);

  int32_t lfo_delta = (params->shape_lfo - s_state.shape_lfo) / (int32_t)frames;

  // s11.20 -> s7.24
  pitch1 <<= 4;
  pitch2 <<= 4;

  // Temporaries.
  uint32_t t1 = s_state.timer1;
  uint32_t t2 = s_state.timer2;
  int32_t shape_lfo = s_state.shape_lfo;

  q31_t * __restrict y = (q31_t *)yn;
  const q31_t * y_e = y + frames;

  // Main Mix/Sub Mix, 8bit(= [0-256])
  const int32_t subVol = (int32_t)(0x100 * s_param.shiftshape);
  const int32_t mainVol = clipmaxi32( 0x160 - subVol, 0x100 );

  const LCWOscWaveSource *src = s_waveSources[s_param.wave];
  for (; y != y_e; ) {
    // input: s7.24 -> s15.16
    const uint32_t dt1 = pitch_to_timer_delta( pitch1 >> 8 );
    const uint32_t dt2 = pitch_to_timer_delta( pitch2 >> 8 );

    // s15.16 -> s11.20
    // x 0.9375(= 15/16)
    int32_t out1 = myLookUp(t1, dt1, src) * 15;
    int32_t out2 = myLookUp(t2, dt2, src) * 15;

    // s11.20 -> s3.28 -> s7.24
    int32_t out = (out1 * mainVol) + (out2 * subVol);
    out = lookupClipCurve( out >> 4 );

    // s7.24 -> q31
    *(y++) = (q31_t)(out << (31 - 24));

    t1 = (t1 + dt1) & LCW_OSC_TIMER_MASK;
    t2 = (t2 + dt2) & LCW_OSC_TIMER_MASK;
    shape_lfo += lfo_delta;
    pitch2 += (shape_lfo >> 16);
  }

  s_state.timer1 = t1;
  s_state.timer2 = t2;
  s_state.shape_lfo = params->shape_lfo;
  s_state.pitch1 = pitch1;
  s_state.pitch2 = pitch2;
}

void OSC_NOTEON(const user_osc_param_t * const params)
{
  return;
}

void OSC_NOTEOFF(const user_osc_param_t * const params)
{
  return;
}

void OSC_PARAM(uint16_t index, uint16_t value)
{
  float valf = param_val_to_f32( value );
  switch (index) {
  case k_user_osc_param_shape:
    s_param.shape = clip01f( valf );
    break;
  case k_user_osc_param_shiftshape:
    s_param.shiftshape = clip01f( valf );
    break;
  case k_user_osc_param_id1:
    s_param.wave = (uint32_t)clipmaxu32(value, 2);
    break;
  default:
    break;
  }
}
