# DSP56k

A Motorola DSP56001 in SystemVerilog, as the Atari Falcon030 uses it: the
core, its on-chip memories and ROMs, the host interface to the 68030, the SSI
link to the Falcon's sound crossbar, and 32K words of external zero-wait-state
RAM. It is the DSP of the [Falcon MiSTer core](https://github.com/apolkosnik/Falcon_MiSTer)
(branch `ap060`), taken out on its own.

The behaviour follows [Hatari](https://framagit.org/hatari/hatari)'s DSP
emulation (`src/falcon/dsp_cpu.c`, `dsp_core.c`, `dsp.c`) function by
function, and the testbench runs Hatari's own code beside the RTL and compares
them instruction by instruction.

## Files

| File | |
|---|---|
| `rtl/falcon_dsp.sv` | top level: the 68030 host port, reset, the SSI slot link, the memories |
| `rtl/dsp56k_core.sv` | instruction execution, AGU, program controller (hardware stack, DO/REP), interrupts, memory map |
| `rtl/dsp56k_alu.sv` | data ALU: MAC, shifter, limiting, scaling, rounding |
| `rtl/dsp56k_dec.sv` | instruction decoder |
| `rtl/dsp_periph.sv` | host interface, SSI, port C, IPR, BCR, SCI and the other peripheral registers |
| `rtl/dsp_ram.sv` | RAM inference templates (Intel M10K) |
| `rtl/dsp56k_rom.sv` | X ROM (mu-law/A-law) and Y ROM (sine), generated |
| `rtl/dsp56k_tables.svh` | opcode dispatch tables, generated |
| `tb/` | Verilator testbench, Hatari golden model, table generator |

`dsp56k_rom.sv` and `dsp56k_tables.svh` are generated from Hatari's sources by
`tb/gen/gen_tables.py`; `tb/run.sh` regenerates them on every run, so the RTL's
decode always matches Hatari's dispatch tables.

## Interface

`falcon_dsp` runs on one clock (32 MHz on the Falcon, `CLK_HZ`):

- **68030 host port**, `$FFA200-$FFA207` on the Falcon: a 16-bit device bus
  (`bus_cs`, `bus_stb`, `bus_we`, `bus_addr[2:1]`, `bus_uds`/`bus_lds`,
  `bus_din`, `bus_dout`, `bus_ack`). Byte accesses as on the hardware: ICR,
  CVR, ISR, IVR, then RXH/TXH, RXM/TXM, RXL/TXL.
- **Interrupt**: `hreq` (ISR.HREQ, level) and `ivr`, the vector the system
  supplies when it acknowledges; `iack` is accepted but unused.
- **Reset**: `dsp_reset`, level (on the Falcon, PSG port A bit 4). After reset
  the DSP takes its 512-word bootstrap through the host port, as the hardware
  does.
- **SSI**: a slot interface to the crossbar (`ssi_slot_stb`, `ssi_frame`,
  `ssi_tx_en`, `ssi_rx_en`, `ssi_rx_frame`, 16-bit `ssi_rx_data` /
  `ssi_tx_data`, `ssi_tx_valid`, `ssi_hs_play_req`). The header of
  `falcon_dsp.sv` gives the timing and how it maps onto Hatari's SSI.

The header of each file lists its behaviour in detail, including the few
places where it follows the hardware documentation instead of Hatari (SWI,
WAIT and STOP do what the manual says; Hatari treats them as NOPs).

## Resources

In the Falcon MiSTer core on a Cyclone V (5CSEBA6, the DE10-Nano), Quartus
17.0: about 5,700 ALMs, 2,200 registers, 104 M10K blocks (96 of them the 32K x
24 external RAM) and one DSP block.

## Testing

```
HATARI=/path/to/hatari tb/run.sh
FALCON=/path/to/Falcon_MiSTer HATARI=/path/to/hatari tb/run.sh   # plus the crossbar test
```

Needs Verilator, gcc and Python 3, and a Hatari source tree that has been
configured with CMake (the build directory with its `config.h`); `HATARI`
defaults to `~/hatari`. Hatari's `dsp_cpu.c` and `dsp_core.c` are compiled
unmodified as the golden model.

The tests:

- the decoder against every entry of Hatari's `opcodes8h[]`;
- the reset state;
- random instruction streams in lockstep with Hatari, every register and
  memory write compared after every instruction (`--seeds N --insns M`);
- directed tests: DO/REP nesting and ENDDO, AGU modulo and bit-reverse
  addressing, limiting/scaling/rounding/MAC, DIV and NORM sequences,
  interrupts (host commands, host receive, SSI, illegal instruction, stack,
  trace), SWI/WAIT/STOP;
- the host port: the 512-word bootstrap, data exchange, HF0, host commands,
  HREQ and IVR;
- the SSI with the crossbar's slot timing;
- with `FALCON` set, the DSP connected to the Falcon core's real
  `falcon_crossbar.sv` in an SSI loopback.

`tb/run.sh --only NAME` runs one test; `--quiet` shortens the output.

## License

GNU General Public License, version 2 or (at your option) any later version.
See `LICENSE`. The design is a translation of Hatari's DSP emulation, which is
licensed the same way, and its ROM and decode tables are generated from
Hatari's sources.
