#pragma once

/*
 * host_clock.h — the desktop host's simulation/presentation pacing clock.
 *
 * Two independent deadlines: when the next guest frame is due, and when the
 * next presented frame is due. They coincide for an ordinary host (present
 * every simulated frame). A host with a custom presenter may present at the
 * display's refresh rate while simulating at the guest's, interpolating
 * between simulated frames; `alpha` is the interpolation weight.
 *
 * Debt policy: a frame that ends up to SNES_HOST_CATCHUP_PERIODS late keeps
 * its phase, and the following frames run back to back until the clock is on
 * time again. Audio is produced in guest time, so every period dropped is a
 * period of samples the device never gets: dropping the debt after each 20-50
 * ms hitch lost ~0.8% of the guest's time on a 165 Hz display, drained the
 * ~67 ms audio cushion every 10-25 s, and each underflow faded the music out
 * and back in. Lateness beyond that window (a load, a window drag, a
 * breakpoint) still drops its debt and re-anchors to now, so the guest never
 * sprints after a real stall. A game may ask to PRESERVE all debt for phases
 * where bursting is the right thing -- Super Metroid's door transitions,
 * whose non-interactive frames refill the guest-driven audio queue -- through
 * the host descriptor's keep_pacing_debt hook.
 *
 * Moved up from SuperMetroidRecomp's sm_video.c, where it was called SmClock
 * and hardwired to that title's frame rate. Nothing in it was Super Metroid.
 */

#include <stdbool.h>
#include <stdint.h>

/* How far behind (in simulation periods) the clock catches up rather than
 * re-anchoring. Matches the audio delivery cushion (RTL_AUDIO_TARGET_NATIVES,
 * four native blocks). */
#define SNES_HOST_CATCHUP_PERIODS 4.0

#ifdef __cplusplus
extern "C" {
#endif

/* NTSC SNES field rate: 21.477272 MHz / (262 * 1364). */
#define SNES_HOST_NTSC_HZ 60.098811862

typedef struct SnesHostClock {
  double next_simulation, next_presentation;
  double simulation_hz, presentation_hz;
  uint64_t simulation_frames, presentations, missed_presentations;
} SnesHostClock;

void   snes_host_clock_reset(SnesHostClock *c, double now,
                             double simulation_hz, double presentation_hz);
bool   snes_host_clock_simulation_due(const SnesHostClock *c, double now);
/* `elapsed_periods` is how many guest frame periods the simulated frame
 * actually spanned (RtlLastFramePeriods()); a lag frame spans more than one. */
void   snes_host_clock_simulation_done(SnesHostClock *c, double now,
                                       bool preserve_debt, double elapsed_periods);
bool   snes_host_clock_presentation_due(const SnesHostClock *c, double now);
void   snes_host_clock_presentation_done(SnesHostClock *c, double now);
double snes_host_clock_alpha(const SnesHostClock *c, double now);
double snes_host_clock_next_deadline(const SnesHostClock *c);

/* Which presentation rate to run at: an explicit fps if it is one the clock
 * supports, else the display's refresh, clamped to a sane range. 0 = display. */
bool   snes_host_valid_fps(unsigned fps);
double snes_host_presentation_hz(unsigned fps, double display_refresh);

#ifdef __cplusplus
}
#endif
