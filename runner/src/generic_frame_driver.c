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
/* Did the CPU half end this frame parked on a hardware wait? */
static int s_took_wai;
static int s_trace_irq = -1;
static int s_replay_irq = -1;
/* Replaying the IRQ during the draw walk exists to put its PPU/OAM/DMA
 * writes on the right scanline. The CPU half already executed the handler
 * once, so the replay must not also commit its guest-RAM side effects:
 * Secret of Mana's intro IRQ advances the background-decompression state,
 * and committing it twice made the tree scene arrive at twice the frame
 * rate. Keep the device writes, restore the guest machine around the
 * replay. */
static uint8_t s_irq_ram[0x20000];
static uint8_t s_irq_vram[0x10000];
static CpuState s_irq_cpu;
static uint32_t s_irq_resume_pc;
/* The proper frame-model raster path: the CPU half journals every display
 * register write as it happens, and the draw walk replays those writes per
 * line. Re-executing the guest IRQ in the draw walk was only ever a stand-in
 * for this journal -- and an unsafe one, because the handler also advances
 * game state and decompresses video data. */
static int s_raster_journal = -1;
static int s_raster_journal_init;

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

/* Env-gated OAM snapshot dump for the frame-model split. The production build
 * compiles the debug server out (SNESRECOMP_TRACE=0), so this is the only way
 * to see which table the draw walk actually consumed. */
static int s_oam_dump_from = -2;
static int s_oam_dump_to = -2;

static void gfd_maybe_dump_oam(void) {
    if (s_oam_dump_from == -2) {
        const char *a = getenv("SNESRECOMP_OAM_DUMP_FROM");
        const char *b = getenv("SNESRECOMP_OAM_DUMP_TO");
        s_oam_dump_from = (a && *a) ? atoi(a) : -1;
        s_oam_dump_to = (b && *b) ? atoi(b) : s_oam_dump_from;
    }
    if (s_oam_dump_from < 0) return;
    int frame = (int)snes_frame_counter;
    if (frame < s_oam_dump_from || frame > s_oam_dump_to) return;

    const char *dir = getenv("SNESRECOMP_DUMP_DIR");
    if (!dir || !*dir) dir = ".";
    char path[1024];
    FILE *f;
    snprintf(path, sizeof(path), "%s\\oam_start_f%d.bin", dir, frame);
    f = fopen(path, "wb");
    if (f) { fwrite(s_oam_field_start, 1, GFD_OAM_BYTES, f); fclose(f); }
    snprintf(path, sizeof(path), "%s\\oam_late_f%d.bin", dir, frame);
    f = fopen(path, "wb");
    if (f) { fwrite(s_oam_field_late, 1, GFD_OAM_BYTES, f); fclose(f); }
    snprintf(path, sizeof(path), "%s\\oam_live_f%d.bin", dir, frame);
    f = fopen(path, "wb");
    if (f) {
        uint8_t live[GFD_OAM_BYTES];
        gfd_capture_oam(live);
        fwrite(live, 1, GFD_OAM_BYTES, f);
        fclose(f);
    }
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
        /* Where does the guest continue?
         *
         * Two shapes exist. The ordinary one ends the handler with an RTI, and
         * the handler's return PC is where execution continues. The other ends
         * it with a JML out of WRAM -- Secret of Mana's task switch, which
         * never returns to the interrupted code at all -- and the interpreter
         * reports the new task's entry point as its resume PC.
         *
         * The exit PC is the LAST fallback, not the first: for the task
         * switch it is the address of the JML itself, and resuming there
         * re-executes the jump outside the interrupt context, where the
         * guard that recognises it is not armed and where the beam does not
         * advance on a $4212 read. The guest then spins in the ROM main
         * loop's vblank poll with a frozen beam for the rest of the session
         * (measured: the name-entry screen never appeared and the sprite
         * table was never uploaded). */
        uint32_t resume = interp_bridge_lle_resume_pc();
        if (!resume) resume = interp_bridge_last_interrupt_return_pc();
        if (!resume) resume = interp_bridge_last_exit_pc();
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
  if (s_raster_journal > 0) {
    ppu_rasterReset();
    s_raster_journal_init = 0;
  }
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
    s_took_wai = 0;
    if (s_trace_irq < 0) {
        const char *e = getenv("SNESRECOMP_TRACE_IRQ");
        /* atoi, not a boolean: 2 asks for the per-slice trace as well. */
        s_trace_irq = (e && *e) ? atoi(e) : 0;
    }
    if (s_replay_irq < 0) {
        const char *e = getenv("SNESRECOMP_IRQ_REPLAY");
        s_replay_irq = (e && *e) ? atoi(e) : 1;
    }
    if (s_raster_journal < 0) {
        const char *e = getenv("SNESRECOMP_RASTER_JOURNAL");
        s_raster_journal = (e && *e) ? (atoi(e) != 0) : 0;
    }

    if (booting) {
        s_resume_pc = reset_vector();
        apply_boot_policy();
    }

    if (s_raster_journal) {
        if (!s_raster_journal_init) {
            ppu_rasterSetHostFieldBoundary(true);
            ppu_rasterBegin(g_ppu);
            s_raster_journal_init = 1;
        }
        /* A frame-model host runs the whole CPU half first, so it owns the
         * field boundary: close the previous field and snapshot the register
         * state this field's guest code starts from. */
        ppu_rasterHostFieldBoundary();
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
        if (interp_bridge_lle_took_wai()) { s_took_wai = 1; break; }
    }

    /* Env-gated per-frame IRQ/beam trace: which frames the CPU half actually
     * took the raster interrupt, and where the beam stood when it did. */
    if (s_trace_irq) {
        static int s_irq_log = -1;
        if (s_irq_log < 0) {
            const char *p = getenv("SNESRECOMP_IRQ_LOG");
            s_irq_log = (p && *p) ? atoi(p) : 0;
        }
        if (s_irq_log) {
            uint32_t wr = 0;
            for (int i = 0; i < 128; i++) {
                if (g_ppu->oam[i] != 0xF0F0u) { wr = (uint32_t)i; break; }
            }
            fprintf(stderr,
                    "[irqlog] f=%d resume=%06X nmi=%d taken=%d slices=%d wai=%d "
                    "vpos=%u vtimer=%u virq=%d I=%d inIrq=%d oam0=%04X "
                    "firstNonF0=%u mc=%llu fend=%llu ",
                    snes_frame_counter, (unsigned)s_resume_pc,
                    (int)g_snes->nmiEnabled,
                    (int)s_irq_taken_this_frame, slice, s_took_wai,
                    (unsigned)g_snes->vPos, (unsigned)g_snes->vTimer,
                    (int)g_snes->vIrqEnabled, (int)g_cpu._flag_I,
                    (int)g_snes->inIrq, (unsigned)g_ppu->oam[0], wr,
                    (unsigned long long)g_cpu.master_cycles,
                    (unsigned long long)frame_end);
        }
    }
}

void snes_generic_frame_driver_draw_ppu_frame(void) {
    Dma *dma = g_snes->dma;
    int trigger, line;
    uint8_t hdmaen;

    /* Restore the field's opening register state before anything walks the
     * HDMA tables: HDMA writes the same registers per line, and restoring
     * after the first line would stomp its first transfer. */
    if (s_raster_journal)
        ppu_rasterRenderBeginCurrent(g_ppu);

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
    gfd_maybe_dump_oam();
    gfd_restore_oam(s_oam_field_start);

    /* The NMI's vblank writes are the field's opening state; the guest ran
     * them before the draw, so replay them from the journal first. */
    if (s_raster_journal)
        ppu_rasterApplyVblank(g_ppu);

    /* From line 0: starting at 1 leaves the top scanline holding the previous
     * frame's state, a stripe of stale tilemap above a HUD. */
    for (line = 0; line <= 224; line++) {
        if (s_trace_irq > 0 && line == trigger)
            fprintf(stderr, "[gfd] f=%d draw line=%d trigger=%d I=%d inIrq=%d taken=%d journal=%d\n",
                    snes_frame_counter, line, trigger, (int)g_cpu._flag_I,
                    (int)g_snes->inIrq, (int)s_irq_taken_this_frame,
                    s_raster_journal);
        if (s_raster_journal) {
            ppu_rasterApplyLine(g_ppu, line);
            if (ppu_rasterTakeHdmaen(&hdmaen))
                dma_startDma(dma, hdmaen, true);
        }
        if (line == trigger) {
            if (s_raster_journal) {
                /* The CPU half already executed the handler once; its late
                 * OAM upload is captured in the snapshot. Put that table back
                 * for the lines below the split without running the handler a
                 * second time. */
                if (s_irq_taken_this_frame)
                    gfd_restore_oam(s_oam_field_late);
                trigger = g_snes->vIrqEnabled ? (int)g_snes->vTimer : -1;
            } else if (s_irq_taken_this_frame && s_replay_irq) {
                const bool isolate_guest = (s_replay_irq >= 2);
                const bool isolate_vram = (s_replay_irq >= 3);
                if (isolate_guest) {
                    memcpy(s_irq_ram, g_ram, sizeof(s_irq_ram));
                    if (isolate_vram)
                        memcpy(s_irq_vram, g_ppu->vram, sizeof(s_irq_vram));
                    s_irq_cpu = g_cpu;
                    s_irq_resume_pc = s_resume_pc;
                }
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
                if (isolate_guest) {
                    memcpy(g_ram, s_irq_ram, sizeof(s_irq_ram));
                    if (isolate_vram)
                        memcpy(g_ppu->vram, s_irq_vram, sizeof(s_irq_vram));
                    g_cpu = s_irq_cpu;
                    s_resume_pc = s_irq_resume_pc;
                }
                /* The interrupt above ran the guest's mid-field code, which
                 * uploaded the sprite table for the lines below this one. Put
                 * that table back: the handler's own DMA wrote the same bytes,
                 * but the snapshot is the one the field actually scanned out. */
                gfd_restore_oam(s_oam_field_late);
                trigger = g_snes->vIrqEnabled ? (int)g_snes->vTimer : -1;
            }
        }
        ppu_runLine(g_ppu, line);
        /* HDMA runs in the H-blank after this line; it lands on the next. */
        dma_doHdma(dma);
    }
    if (s_raster_journal)
        ppu_rasterRenderEnd(g_ppu);
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
