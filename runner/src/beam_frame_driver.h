#ifndef SNESRECOMP_BEAM_FRAME_DRIVER_H
#define SNESRECOMP_BEAM_FRAME_DRIVER_H

/* Beam-aligned frame driver for whole-program LLE ports.
 *
 * One host frame is one PPU field, cut where hardware raises NMI: the beam's
 * vblank edge (V=225). The CPU half runs the guest against the real beam,
 * taking every interrupt where it latches -- raster H/V matches and a
 * coprocessor holding the IRQ line (SuperFX, Cx4, SA-1) -- and lets hardware
 * time pass while the guest is parked, so the beam, APU and coprocessors get
 * the whole field. The render half rasterizes that field from the raster
 * journal (ppu.c), stepping the real HDMA unit per line.
 *
 * new_project scaffolds call these from src/game_rtl.c; a title replaces
 * them only when it needs something they do not do (see
 * docs/FRAME_MODEL_HOSTS.md). */

void snes_beam_frame_driver_run_frame(void);
void snes_beam_frame_driver_draw_ppu_frame(void);
/* Observe the display state that actually produced each visible line, after
 * raster journal writes and before H-blank HDMA. The PPU is read-only and the
 * callback must not advance the machine or change guest memory/registers.
 * `line` is 1..224, corresponding to output row line-1. No observer is kept
 * between calls; the regular draw entry point is identical to passing NULL. */
struct Ppu;
typedef void SnesBeamRasterObserver(const struct Ppu *ppu, unsigned line, void *context);
void snes_beam_frame_driver_draw_ppu_frame_observed(
    SnesBeamRasterObserver *observer, void *context);
/* Forget the booted machine: the next frame starts from the reset vector. */
void snes_beam_frame_driver_reset(void);
/* A save state was restored: resume where the restored machine stands. */
void snes_beam_frame_driver_state_loaded(void);
/* The driver's own host state -- where the CPU half resumes, and whether it is
 * halted on WAI -- for a title that saves it beside its snapshot. A guest blob
 * alone does not carry it, so a load otherwise resumes the restored machine at
 * the PC of the session it replaced. */
#include <stdbool.h>
#include <stdint.h>
void snes_beam_frame_driver_get_resume(uint32_t *pc, bool *wai_halted);
void snes_beam_frame_driver_set_resume(uint32_t pc, bool wai_halted);

#endif /* SNESRECOMP_BEAM_FRAME_DRIVER_H */
