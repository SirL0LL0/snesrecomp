#include "generic_frame_driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common_rtl.h"          /* g_snesrecomp_last_hdmaen */
#include "cpu_state.h"
#include "snes/dma.h"
#include "snes/snes.h"
#include "snes/interp_bridge.h"
#include "snes/ppu.h"
#include "snes/snes.h"

/* One NTSC frame: 262 scanlines x 1364 master clocks. Bounds a productive
 * MMIO loop so it cannot run across several vblanks atomically. */
#define GFD_MASTER_CYCLES_PER_FRAME 357368ull

/* How many times a frame may re-enter the guest after it parks on a poll.
 * Each slice ends at a deterministic read-only cycle or at an IRQ; the bound
 * only stops a pathological loop from spinning the host. */
#define GFD_MAX_SLICES_PER_FRAME 64

/* 0 until the first frame has booted from the reset vector. */
static uint32_t s_resume_pc;
static SnesGenericBootPolicy s_boot;
static RtlGameInfo s_game_info;
static char s_title[64] = "variant";

/* Did the CPU half actually take the V/H comparator IRQ this frame? The draw
 * loop replays that interrupt at the trigger line to raster the register and
 * OAM writes at the right scanline, but a replay is only valid if the guest
 * really entered the handler. When the guest keeps I set across the whole
 * field (SoM's prologue does), the CPU half leaves the IRQ pending, and
 * forcing it in the draw loop runs code hardware never ran -- $C1CF writes
 * $212C=$14 there and cuts a flat band into the bottom of the field. */
static bool s_irq_taken_this_frame;
static int s_trace_irq = -1;

void snes_generic_frame_driver_set_boot_policy(const SnesGenericBootPolicy *policy) {
    if (policy) s_boot = *policy;
    else memset(&s_boot, 0, sizeof(s_boot));
    if (s_boot.poke_count > 8) s_boot.poke_count = 8;
}

const SnesGenericBootPolicy *snes_generic_frame_driver_boot_policy(void) {
    return &s_boot;
}

/* The sprite table a field starts with, and the one the guest uploads
 * mid-field.
 *
 * Hardware scans the OAM out as the beam passes it: the table uploaded
 * before the field's first line draws the lines above that upload, the
 * table uploaded later draws the lines below it. SoM uploads the sprite
 * table twice -- once from the NMI, which opens the field, and once from
 * the raster interrupt at line 192 -- and the second upload is what the
 * lines below the split draw. The tables differ: the NMI's is the same
 * every frame, the mid-field one carries the current sprite state.
 *
 * A frame-model host runs the whole CPU half before it draws anything, so
 * by the time the first line is rastered the OAM already holds the late
 * upload: without this the entire field would draw the new sprites and the
 * upper half would lose the old ones. */
#define GFD_OAM_BYTES 544
static uint8_t s_oam_field_start[GFD_OAM_BYTES];
static uint8_t s_oam_field_late[GFD_OAM_BYTES];

static void gfd_capture_oam(uint8_t *dst) {
    memcpy(dst, g_ppu->oam, 512);
    memcpy(dst + 512, g_ppu->highOam, 32);
}

static void gfd_restore_oam(const uint8_t *src) {
    memcpy(g_ppu->oam, src, 512);
    memcpy(g_ppu->highOam, src + 512, 32);
}

static uint32_t read_vector(uint32_t addr) {
    /* Read through the guest bus so a mapper or coprocessor window resolves
     * the same way the CPU sees it. */
    uint32_t lo = snes_read(g_snes, addr);
    uint32_t hi = snes_read(g_snes, addr + 1u);
    return (hi << 8) | lo;
}

static uint32_t reset_vector(void) { return read_vector(0x00FFFCu); }
static uint32_t nmi_vector(void)   { return read_vector(0x00FFEAu); }
static uint32_t irq_vector(void)   { return read_vector(0x00FFEEu); }

/* Run one interrupt handler to its RTI, entered as hardware enters it: the
 * frame is pushed at the PC the guest was interrupted AT, so the handler's
 * terminal RTI returns into that instruction stream. */
static void run_interrupt(uint32_t vector, uint64_t frame_end) {
    cpu_push_interrupt_frame_at(&g_cpu, s_resume_pc);
    interp_bridge_clear_lle_resume_pc();
    interp_bridge_set_master_deadline(frame_end);
    (void)interp_bridge_run_interrupt(&g_cpu, vector);
    /* Clearing matters: a deadline left armed stays true for every AOT block
     * prologue afterwards, which turns every compiled body into an immediate
     * yield-unwind. */
    interp_bridge_set_master_deadline(0);
    {
        uint32_t resume = interp_bridge_last_interrupt_return_pc();
        if (!resume) resume = interp_bridge_last_exit_pc();
        if (!resume) resume = interp_bridge_lle_resume_pc();
        if (resume) s_resume_pc = resume;
    }
}

static void apply_boot_policy(void) {
    for (unsigned i = 0; i < s_boot.poke_count; i++) {
        uint32_t addr = s_boot.pokes[i].addr24 & 0xFFFFFFu;
        snes_write(g_snes, addr, s_boot.pokes[i].value);
    }
    if (s_boot.entry_pc24) s_resume_pc = s_boot.entry_pc24 & 0xFFFFFFu;
    if (s_boot.poke_count || s_boot.entry_pc24)
        fprintf(stderr, "[frame-driver] boot policy: %u poke(s), entry $%06X\n",
                s_boot.poke_count, s_resume_pc);
}

/* A state load (rewind commit, save-state browser, netplay sync) restores
 * the guest's registers and memory, but the resume PC this driver keeps is
 * HOST state: it is not part of the guest blob. Left alone it still points at
 * the address the guest was running BEFORE the snapshot, so the machine
 * resumes somewhere the restored memory does not describe -- the rewind
 * looked like a hang. Take the resume PC from the restored interpreter state
 * instead (the rollback blob carries it), falling back to the last exit PC. */
void snes_generic_frame_driver_state_loaded(void) {
  uint32_t resume = interp_bridge_lle_resume_pc();
  if (!resume) resume = interp_bridge_last_exit_pc();
  if (resume) s_resume_pc = resume;
}

void snes_generic_frame_driver_run_frame(void) {
    /* HDMA ownership. This driver walks the HDMA tables in its own
     * raster loop (dma_doHdma, one transfer per visible line),
     * which is what hardware does. The framework's beam HDMA would
     * ALSO walk them, from snes_advance_beam during the CPU half of
     * the frame -- consuming the table before the draw loop reaches it,
     * so the channel transferred nothing and the guest never saw the
     * transfer complete. SoM's intro decompresses its background
     * through HDMA and waits for that completion: with the tables
     * eaten early the wait never ends and the image fills in over
     * thousands of frames instead of hundreds.
     *
     * Claim the tables for the whole frame, before the guest runs. */
    if (g_snes) snes_set_hdma_beam_enabled(g_snes, false);
    const uint64_t frame_end = g_cpu.master_cycles + GFD_MASTER_CYCLES_PER_FRAME;
    const int booting = (s_resume_pc == 0);
    int slice;

    s_irq_taken_this_frame = false;
    if (s_trace_irq < 0) {
        const char *e = getenv("SNESRECOMP_TRACE_IRQ");
        s_trace_irq = (e && *e) ? 1 : 0;
    }

    if (booting) {
        s_resume_pc = reset_vector();
        apply_boot_policy();
    }

    /* Vblank edge. NMITIMEN gates it: delivering before the guest has enabled
     * NMI would land an interrupt frame in the middle of its SEI boot
     * sequence. Nothing is delivered on the very first frame either -- reset
     * has not run yet, so there is no instruction stream to interrupt. */
    if (!booting && g_snes->nmiEnabled) {
        g_snes->inNmi = true;
        run_interrupt(nmi_vector(), frame_end);
        g_snes->inNmi = false;
    }

    /* The field's opening sprite table: what the NMI left in the OAM when
     * the field began. The lines above the mid-field upload draw this. */
    gfd_capture_oam(s_oam_field_start);

    /* Run the guest until it parks on a read-only poll (its vblank wait) or
     * the frame's clock runs out. The guest typically parks several times per
     * frame -- on HVBJOY, on a DMA-complete flag, on its own state machine --
     * and each park needs either an interrupt or simply more time. */
    for (slice = 0; slice < GFD_MAX_SLICES_PER_FRAME; slice++) {
        if (g_cpu.master_cycles >= frame_end) break;
        interp_bridge_set_master_deadline(frame_end);
        interp_bridge_run_until_quiescent(&g_cpu, s_resume_pc);
        interp_bridge_set_master_deadline(0);
        {
            uint32_t resume = interp_bridge_lle_resume_pc();
            if (resume) s_resume_pc = resume;
        }
        /* Bring the beam up to the CPU's clock. A recompiled guest charges
         * the master clock itself (cpu->master_cycles += N) and never reaches
         * the device layer, so without this the beam only creeps -- 18 dots in
         * a whole field -- and everything that reads it sees a machine
         * standing still: $4212's vblank bit, $2137's counters. The host is
         * the only place that holds both clocks. */
        snes_sync_master_clock(g_snes, g_cpu.master_cycles);
        /* A raster IRQ asserted while the guest ran: service it before
         * continuing, exactly as the CPU samples it between instructions. */
        if (g_snes->inIrq && !g_cpu._flag_I) {
            s_irq_taken_this_frame = true;
            if (s_trace_irq)
                fprintf(stderr, "[gfd] f=%d cpu-irq vpos=%u vtimer=%u\n",
                        snes_frame_counter, (unsigned)g_snes->vPos,
                        (unsigned)g_snes->vTimer);
            run_interrupt(irq_vector(), frame_end);
            continue;
        }
        /* Parked with no interrupt pending and clock left over: the guest is
         * waiting for the next vblank. Nothing more happens this frame. */
        if (interp_bridge_lle_took_wai()) break;
    }
}

void snes_generic_frame_driver_draw_ppu_frame(void) {
    Dma *dma = g_snes->dma;
    int trigger, line;

    /* Re-arm HDMA from the last $420C (HDMAEN) the guest wrote -- typically
     * during the NMI just run. The framework records it for exactly this. */
    dma_startDma(dma, g_snesrecomp_last_hdmaen, true);
    /* HDMA ownership. This is a frame-model host: the raster loop below
     * walks the tables itself, once per visible line, which is what
     * hardware does. The framework ships that engine (dma_initHdma +
     * dma_doHdma) precisely for this case -- snes.h says so -- and the
     * simplified SimpleHdma is NOT it: it keeps no table pointer in the
     * channel, so $43n8/$43n9 (A2An, the HDMA table's current address)
     * would read back something the guest never wrote. A guest that
     * reads A2An to know where its table is -- and a channel switched on
     * mid-field restarts its table, which moves A2An -- then takes a
     * different path and the whole effect shifts. The real unit keeps
     * tableAdr in step with the transfer, so A2An reads what hardware
     * would report.
     *
     * The beam must not also step the tables from inside guest register
     * writes, or they are consumed twice per frame. */
    snes_set_hdma_beam_enabled(g_snes, false);
    /* The field's register state as of its start: every enabled channel
     * reloads its table pointer, exactly as the HDMA controller re-reads the
     * configuration at scanline 0. */
    dma_initHdma(dma);
    /* Hardware's V=0 init also performs each channel's first transfer
     * before any visible pixel. */
    dma_primeHdmaFirstLine(dma);

    /* Mid-frame raster split, if the guest programmed the V comparator. */
    trigger = g_snes->vIrqEnabled ? (int)g_snes->vTimer : -1;

    /* The sprite table as the guest left it -- the one the mid-field
     * interrupt uploaded -- and the field's opening table back in place.
     * The lines above the upload draw the opening table, the ones below it
     * draw the late one. */
    gfd_capture_oam(s_oam_field_late);
    gfd_restore_oam(s_oam_field_start);

    /* From line 0: starting at 1 leaves the top scanline holding the previous
     * frame's state, a stripe of stale tilemap above a HUD. */
    for (line = 0; line <= 224; line++) {
        if (s_trace_irq > 0 && line == trigger)
            fprintf(stderr, "[gfd] f=%d draw line=%d trigger=%d I=%d inIrq=%d taken=%d\n",
                    snes_frame_counter, line, trigger, (int)g_cpu._flag_I,
                    (int)g_snes->inIrq, (int)s_irq_taken_this_frame);
        if (line == trigger && s_irq_taken_this_frame) {
            g_snes->inIrq = true;
            cpu_push_interrupt_frame_at(&g_cpu, s_resume_pc);
            interp_bridge_clear_lle_resume_pc();
            (void)interp_bridge_run_interrupt(&g_cpu, irq_vector());
            g_snes->inIrq = false;
            {
                uint32_t resume = interp_bridge_last_interrupt_return_pc();
                if (!resume) resume = interp_bridge_last_exit_pc();
                if (!resume) resume = interp_bridge_lle_resume_pc();
                if (resume) s_resume_pc = resume;
            }
            /* The interrupt above ran the guest's mid-field code, which
             * uploaded the sprite table for the lines below this one. Put
             * that table back: the handler's own DMA wrote the same bytes,
             * but the snapshot is the one the field actually scanned out. */
            gfd_restore_oam(s_oam_field_late);
            trigger = g_snes->vIrqEnabled ? (int)g_snes->vTimer : -1;
        }
        ppu_runLine(g_ppu, line);
        /* HDMA runs in the H-blank after this line; it lands on the next. */
        dma_doHdma(dma);
    }
}

void snes_generic_frame_driver_reset(void) {
    /* A new session must boot again: the resume PC is the only sticky state
     * this driver owns, and it must not survive snes_free/SnesInit. */
    s_resume_pc = 0;
}

const RtlGameInfo *snes_generic_frame_driver_game_info(const char *title) {
    memset(&s_game_info, 0, sizeof(s_game_info));
    if (title && title[0]) snprintf(s_title, sizeof(s_title), "%s", title);
    s_game_info.title = s_title;
    s_game_info.run_frame = &snes_generic_frame_driver_run_frame;
    s_game_info.draw_ppu_frame = &snes_generic_frame_driver_draw_ppu_frame;
    s_game_info.save_name_prefix = "save";
    s_game_info.session_reset = &snes_generic_frame_driver_reset;
    s_game_info.on_state_loaded = &snes_generic_frame_driver_state_loaded;
    return &s_game_info;
}
