// Falcon DSP testbench - lockstep differential engine. Plain ASCII.
#include "tb_lockstep.h"

// ---------------------------------------------------------------------------
// documented deviation: bit-reverse addressing (Mn = 0) with a zero
// modifier (Nn = 0 for (Rn)+Nn, (Rn)-Nn, (Rn+Nn)).  Hatari's
// dsp_update_rn_bitreverse finds no set bit in the modifier and reverses 17
// bits, which puts bit 16 into Rn, into the (Rn+Nn) address and into
// anything the same instruction copies from Rn (move r2,y:(r2)+n2 stores
// $01xxxx).  The hardware's reverse-carry add of 0 leaves Rn unchanged, which
// is also what Hatari's linear and modulo updates do with a zero modifier.
// An instruction after which Hatari holds a 17-bit Rn is run again with its
// address generator seeing a non-zero Mn for that Rn, while all other reads of
// Mn (moves, MOVEC, ...) still see 0:
//   - through the AGU pipeline slot agu_pipeline_reg/val[0] (read only by
//     dsp_calc_ea, and overwritten at the end of every instruction), when it
//     is free or already holds Mn;
//   - otherwise by setting registers[Mn] for the instruction and restoring
//     it afterwards.  That is invisible only if the instruction does not use
//     Mn as data, so the instruction runs twice, with $FFFF and $7FFF; equal
//     results prove it.  If they differ, Hatari runs unmodified and the
//     instruction is counted as unmodelled (Rn masked to 16 bits).
// ---------------------------------------------------------------------------
static uint32_t agu_now(int reg) {
    return dsp_core.agu_pipeline_reg[0] == (uint32_t)reg ? dsp_core.agu_pipeline_val[0]
                                                         : dsp_core.registers[reg];
}

struct GoldenSnap {
    dsp_core_t core;
    int sc1, sc2, hs;
    uint32_t sc2last;
    void save() { memcpy(&core, &dsp_core, sizeof core); sc1 = golden_sc1_calls; sc2 = golden_sc2_calls;
                  hs = golden_hs_record_frame; sc2last = golden_sc2_last; }
    void load() const { memcpy(&dsp_core, &core, sizeof core); golden_sc1_calls = sc1; golden_sc2_calls = sc2;
                        golden_hs_record_frame = hs; golden_sc2_last = sc2last; }
    bool same() const { return !memcmp(&core, &dsp_core, sizeof core) && sc1 == golden_sc1_calls &&
                               sc2 == golden_sc2_calls && hs == golden_hs_record_frame &&
                               sc2last == golden_sc2_last; }
};
static GoldenSnap snap_pre, snap_a;

// execute one instruction with the M registers in reg_m[] presented as mval
// in registers[] (restored afterwards unless the instruction changed them)
static void exec_reg_subst(const int *reg_m, int n, uint32_t mval) {
    for (int k = 0; k < n; k++) dsp_core.registers[reg_m[k]] = mval;
    dsp56k_execute_instruction();
    for (int k = 0; k < n; k++) {
        if (dsp_core.registers[reg_m[k]] == mval) dsp_core.registers[reg_m[k]] = 0;
        // a move into Mn saved the old (substituted) value for the AGU delay slot
        if (dsp_core.agu_pipeline_reg[0] == (uint32_t)reg_m[k] && dsp_core.agu_pipeline_val[0] == mval)
            dsp_core.agu_pipeline_val[0] = 0;
    }
}

static void golden_execute(Stats &st) {
    bool cand = false;
    for (int i = 0; i < 8; i++)
        if (agu_now(DSP_REG_M0 + i) == 0 && (agu_now(DSP_REG_N0 + i) & 0xffff) == 0) cand = true;
    if (!cand) { dsp56k_execute_instruction(); return; }
    // run Hatari as it is; a 17-bit Rn shows which registers took the
    // zero-modifier bit-reverse update ((Rn)+ / (Rn)- with Mn = 0 are
    // regular bit-reverse updates by 1 and stay as they are)
    snap_pre.save();
    dsp56k_execute_instruction();
    int leak[8], nleak = 0;
    for (int i = 0; i < 8; i++)
        if (dsp_core.registers[DSP_REG_R0 + i] > 0xffff) leak[nleak++] = i;
    if (!nleak) return;
    st.dev_bitrev0++;
    snap_pre.load();
    int rest[8], nrest = 0;
    for (int k = 0; k < nleak; k++) {
        uint32_t m = DSP_REG_M0 + leak[k];
        if (dsp_core.agu_pipeline_reg[0] == m) dsp_core.agu_pipeline_val[0] = 0xffff;
        else if (dsp_core.agu_pipeline_reg[0] == 0) { dsp_core.agu_pipeline_reg[0] = m; dsp_core.agu_pipeline_val[0] = 0xffff; }
        else rest[nrest++] = m;
    }
    if (!nrest) { dsp56k_execute_instruction(); return; }
    snap_a.save();                      // pre-state with the pipeline injection
    exec_reg_subst(rest, nrest, 0xffff);
    static GoldenSnap res_ffff;
    res_ffff.save();
    snap_a.load();
    exec_reg_subst(rest, nrest, 0x7fff);
    if (res_ffff.same()) return;
    // Mn used as data by this instruction: run Hatari unmodified
    st.dev_bitrev0_unmodelled++;
    snap_pre.load();
    dsp56k_execute_instruction();
    for (int i = 0; i < 8; i++) dsp_core.registers[DSP_REG_R0 + i] &= 0xffff;
}


void Lockstep::start(const std::vector<uint32_t> &p, const uint32_t *xint, const uint32_t *yint,
                     const std::vector<uint32_t> &ext, const DspState *regs) {
    Vfalcon_dsp *top = rtl.top;
    rtl.reset();
    golden_reset();
    // state that neither Hatari's dsp_core_reset nor the DSP reset clears
    // (power-up leftovers): give the golden model the RTL's values
    dsp_core.ssi.transmit_value = RP(ssi_tval);
    dsp_core.ssi.received_value = RP(ssi_rval);
    dsp_core.hostport[CPU_HOST_RXH] = RP(rxh); dsp_core.hostport[CPU_HOST_RXM] = RP(rxm);
    dsp_core.hostport[CPU_HOST_RXL] = RP(rxl);
    dsp_core.hostport[CPU_HOST_TXH] = RP(txh); dsp_core.hostport[CPU_HOST_TXM] = RP(txm);
    dsp_core.hostport[CPU_HOST_TXL] = RP(txl);
    dsp_core.interrupt_IplToRaise = RC(int_ipl);
    dsp_core.pc_on_rep = RC(pc_on_rep);
    golden_hs_record_frame = 0;
    hs_play_pulses = 0;
    // memories (identical backdoor load into both models)
    for (int i = 0; i < 32768; i++) {
        uint32_t v = ext[i] & 0xffffff;
        rtl.ext_write(i, v);
        dsp_core.ramext[i] = v;
    }
    for (int i = 0; i < 0x200; i++) {
        uint32_t v = p[i] & 0xffffff;
        RC(u_pint__DOT__mem)[i] = v;
        dsp_core.ramint[DSP_SPACE_P][i] = v;
    }
    for (int i = 0; i < 256; i++) {
        rtl.xyint_write(0, i, xint[i]);
        rtl.xyint_write(1, i, yint[i]);
        dsp_core.ramint[DSP_SPACE_X][i] = xint[i] & 0xffffff;
        dsp_core.ramint[DSP_SPACE_Y][i] = yint[i] & 0xffffff;
    }
    rtl.eval0();
    // start execution like the 68030 does after (part of) a bootstrap: ICR.HF0
    host_write_both(rtl, 0, 0x08);
    host_write_both(rtl, 0, 0x00);
    // wait for the fetch state, then set the registers in both models
    for (int i = 0; i < 10 && rtl.state() != 1; i++) { rtl.eval0(); rtl.clock(); }
    if (rtl.state() != 1) printf("FAIL: DSP did not start after ICR.HF0 (expected fetch state)\n");
    if (regs) {
        rtl.set_regs(*regs);
        golden_set_regs(*regs);
    }
    first = true;
}

int Lockstep::step(const HostEv *h, const SsiEv *s, uint16_t *host_rd, uint16_t *ssi_tx) {
    Vfalcon_dsp *top = rtl.top;
    uint32_t w = golden_pread(dsp_core.pc);
    if (insn_is_hw_deviation(w)) { st.stop_hw_insn++; return -1; }
    {
        std::string c = hatari_class(w);
        st.classes[c]++;
        if (w >= 0x100000 || c == "dsp_pm_0") st.alu_ops[hatari_alu(w)]++;
    }

    // ---------------- golden ----------------
    uint32_t prev_state = dsp_core.interrupt_state;
    uint32_t prev_crb = dsp_core.periph[DSP_SPACE_X][DSP_SSI_CRB];
    golden_execute(st);
    // documented deviation: enabling TE / RE re-arms the wait for the next
    // frame sync (hardware); Hatari's CRB write overwrites the old value
    // before dsp_core_ssi_configure() compares it, so it never re-arms
    {
        uint32_t crb = dsp_core.periph[DSP_SPACE_X][DSP_SSI_CRB];
        bool te_up = !(prev_crb & (1 << DSP_SSI_CRB_TE)) && (crb & (1 << DSP_SSI_CRB_TE));
        bool re_up = !(prev_crb & (1 << DSP_SSI_CRB_RE)) && (crb & (1 << DSP_SSI_CRB_RE));
        if (te_up) dsp_core.ssi.waitFrameTX = 1;
        if (re_up) dsp_core.ssi.waitFrameRX = 1;
        if (te_up || re_up) st.dev_crb_rearm++;
    }
    uint32_t gcyc = dsp_core.instr_cycle;
    if (dsp_core.interrupt_state == DSP_INTERRUPT_DISABLED && dsp_core.interrupt_pipeline_count == 5)
        st.interrupts++;
    if (dsp_core.interrupt_state == DSP_INTERRUPT_LONG && prev_state != DSP_INTERRUPT_LONG)
        st.long_interrupts++;
    // golden_execute() models bit-reverse with a zero modifier; a 17-bit Rn
    // left over means that model missed a case (counted as a mismatch)
    int bitrev_missed = 0;
    for (int i = 0; i < 8; i++)
        if (dsp_core.registers[DSP_REG_R0 + i] > 0xffff) {
            printf("  FAIL: Hatari left R%d = %06x (17-bit bit-reverse result not modelled)\n",
                   i, dsp_core.registers[DSP_REG_R0 + i]);
            st.dev_bitrev0_missed++;
            bitrev_missed++;
            dsp_core.registers[DSP_REG_R0 + i] &= 0xffff;
        }
    // events after the instruction
    uint16_t g_tx = 0, g_rd = 0;
    if (s && s->valid) {
        st.ssi_events++;
        if (s->tx_en) {
            // documented deviation: a frame sync ends the wait for a frame in
            // normal mode too (Hatari: network mode only)
            if (s->frame && !dsp_core.ssi.crb_mode && dsp_core.ssi.waitFrameTX) {
                dsp_core.ssi.waitFrameTX = 0;
                st.dev_ssi_normal_fs++;
            }
            dsp_core_ssi_Receive_SC2(s->frame ? 1 : 0);
            dsp_core_ssi_Receive_SCK();
            g_tx = dsp_core.ssi.transmit_value & 0xffff;
            golden_hs_record_frame = 0;
        }
        if (s->rx_en) {
            uint16_t d = s->loopback ? g_tx : s->rx_data;
            dsp_core.ssi.received_value = (uint32_t)(int32_t)(int16_t)d & 0xffffff;
            if (s->rx_frame && !dsp_core.ssi.crb_mode && dsp_core.ssi.waitFrameRX) {
                dsp_core.ssi.waitFrameRX = 0;
                st.dev_ssi_normal_fs++;
            }
            dsp_core_ssi_Receive_SC1(s->rx_frame ? 1 : 0);
            dsp_core_ssi_Receive_SC0();
            // documented deviation: Hatari's SHFD bit swap can leave a 25th
            // bit in ssi.RX (a value no 24-bit register can hold); the RTL
            // keeps the low 24 bits
            if (dsp_core.ssi.RX > 0xffffff) { dsp_core.ssi.RX &= 0xffffff; st.dev_rx25++; }
        }
    }
    if (h && h->valid) {
        st.host_events++;
        int o = h->off;
        if (h->we) {
            if (h->word) { dsp_core_write_host(o, h->data >> 8); dsp_core_write_host(o + 1, h->data & 0xff); }
            else dsp_core_write_host(o, (o & 1) ? (h->data & 0xff) : (h->data >> 8));
        } else {
            if (h->word) { uint8_t hi = dsp_core_read_host(o); uint8_t lo = dsp_core_read_host(o + 1); g_rd = (hi << 8) | lo; }
            else { uint8_t b = dsp_core_read_host(o); g_rd = (o & 1) ? b : (uint16_t)(b << 8); }
        }
    }

    // ---------------- RTL ----------------
    int clocks = 0;
    uint32_t rcyc = 0;
    uint16_t r_tx = 0, r_rd = 0;
    for (;;) {
        rtl.eval0();
        bool el = rtl.is_el();
        if (el) {
            rcyc = rtl.ncyc();
            if (s && s->valid) {
                top->ssi_slot_stb = 1; top->ssi_tx_en = s->tx_en; top->ssi_rx_en = s->rx_en;
                top->ssi_frame = s->frame; top->ssi_rx_frame = s->rx_frame; top->ssi_rx_data = s->rx_data;
                rtl.eval0();
                if (s->loopback) { top->ssi_rx_data = top->ssi_tx_data; rtl.eval0(); }
                r_tx = top->ssi_tx_data;
            }
            if (h && h->valid) {
                int o = h->off;
                top->bus_cs = 1; top->bus_stb = 1; top->bus_we = h->we; top->bus_addr = (o >> 1) & 3;
                top->bus_uds = h->word || !(o & 1); top->bus_lds = h->word || (o & 1);
                top->bus_din = h->data;
                rtl.eval0();
                if (!top->bus_ack) printf("  FAIL: bus_ack missing on the bus_stb clock\n");
                r_rd = top->bus_dout;
                if (!h->word) r_rd = (o & 1) ? (r_rd & 0xff) : (r_rd & 0xff00);
            }
        }
        rtl.clock();
        clocks++;
        if (top->ssi_hs_play_req) hs_play_pulses++;
        if (el) {
            rtl.idle_inputs();
            rtl.eval0();
            break;
        }
        if (clocks > 400) {
            printf("  FAIL: RTL did not complete the instruction at P:%04x (%06x) within 400 clocks\n",
                   dsp_core.pc, w);
            return -2;
        }
    }

    // ---------------- compare ----------------
    st.insns++;
    char ctx[96];
    snprintf(ctx, sizeof ctx, "insn #%llu %06x (%s)", (unsigned long long)st.insns, w, hatari_class(w));
    bool v = verbose && reported < max_report;
    int n = bitrev_missed;
    if (rcyc != gcyc) {
        st.cyc_mismatch++;
        if (v) printf("  MISMATCH %s: cycles expected (Hatari) %u, RTL %u\n", ctx, gcyc, rcyc);
        n++;
    } else st.cyc_match++;
    if (!first && (uint32_t)clocks != rcyc) {
        if (v) printf("  MISMATCH %s: instruction took %d clocks, ncyc says %u\n", ctx, clocks, rcyc);
        n++;
    }
    first = false;
    DspState gs, rs;
    golden_read_state(gs);
    rtl.read_state(rs);
    n += compare_state(gs, rs, ctx, v);
    if (mem_every <= 1 || (++nstep % mem_every) == 0) n += compare_memories(rtl, ctx, v);
    if (s && s->valid && s->tx_en && g_tx != r_tx) {
        if (v) printf("  MISMATCH %s: SSI transmitted word expected (Hatari) %04x, RTL %04x\n", ctx, g_tx, r_tx);
        n++;
    }
    if (h && h->valid && !h->we && g_rd != r_rd) {
        if (v) printf("  MISMATCH %s: host read at $FFA20%d expected (Hatari) %04x, RTL %04x\n", ctx, h->off, g_rd, r_rd);
        n++;
    }
    if ((int)top->ssi_tx_valid != golden_hs_record_frame) {
        if (v) printf("  MISMATCH %s: ssi_tx_valid expected (Hatari handshake frame) %d, RTL %d\n",
                      ctx, golden_hs_record_frame, top->ssi_tx_valid);
        n++;
    }
    if (hs_play_pulses != golden_sc1_calls) {
        if (v) printf("  MISMATCH %s: ssi_hs_play_req pulses expected (Hatari SC1 calls) %d, RTL %d\n",
                      ctx, golden_sc1_calls, hs_play_pulses);
        n++;
    }
    if (host_rd) *host_rd = r_rd;
    if (ssi_tx) *ssi_tx = r_tx;
    if (n) {
        st.mismatching_insns++;
        if (v) {
            reported++;
            printf("  (golden PC after: %04x, RTL PC %04x, SR %04x/%04x)\n", gs.pc, rs.pc, gs.sr, rs.sr);
        }
    }
    return n;
}
