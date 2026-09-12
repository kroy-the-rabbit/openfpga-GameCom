// Copyright (c) 2026 Jamie Blanks

//============================================================================
// Sharp SM8521 - the Game.com's CPU, with its on-chip DMA blitter, LCDC,
// sound and timers.
//
// Every register is clocked by clk_sys. phi0_ce_i and phi1_ce_i are enables,
// not clocks. One target cycle - a "beat" - is four clk_sys ticks, and the
// work inside it runs in this order:
//
//   tick 0   phi0_ce   start of beat, then bus and the ST_* state machine
//   tick 1
//   tick 2   phi1_ce   decode and execute, then end of beat
//   tick 3
//
// Start of beat clears last beat's requests and reads the operands the
// execute stage might need. End of beat runs apply_bus_request,
// apply_addr_request and apply_pending_writes.
//
// The rule that keeps this file small: the state machine never performs an
// access, it records one. Every bus read and write, every direct-address
// write and every effective-address calculation is a request that a single
// drain carries out at the end of the beat. That is why the SFR decode is
// built a handful of times instead of at each of its call sites, and it is
// the first thing to preserve when changing this file.
//
// The CKC prescaler can run the core slower than phi0; peripherals always
// use the unprescaled phi0_ce_i.
//============================================================================

`include "rtl/sm8521_decode.v"
`include "rtl/sm8521_gp_store.v"
`include "rtl/sm8521_sound.v"
`include "rtl/sm8521_timers.v"

module sm8521
#(
	// Hardware reset warm-up is 2^18 main-clock periods. This core decrements
	// on phi1, so the production default is the fCK/2 half-count.
	parameter [17:0] RESET_WARMUP_CYCLES = 18'd131072,
	// The real Game.com core runs phi0 at 5 MHz. Timer, LCDC, DMA, watchdog,
	// and sound-generator cadence are modeled directly in those phi0 ticks.
	parameter integer CLKT_1S_PHI0_CYCLES = 5000000,
	parameter integer UART_TX_BIT_PHI0_CYCLES = 32,
	parameter integer WDT_FC12_PHI0_CYCLES = 4096,
	parameter integer WDT_FX5_PHI0_CYCLES = ((CLKT_1S_PHI0_CYCLES < 1024) ? 1 : (CLKT_1S_PHI0_CYCLES >> 10)),
	parameter [24:0] CLK_SYS_HZ = 25'd20000000,
	parameter [24:0] SUBCLOCK_HZ = 25'd32768,
	// Deliberate divergence from the datasheet, kept on purpose.
	//
	// Real hardware resets CKC to 00H and runs the CPU at fCK/32 until software
	// arms FCPUEN and executes a STOP, which is what actually commits a new
	// FCPUS. Patched BIOS paths here can skip that STOP, so the core would sit
	// at fCK/32 forever and boot sixteen times too slowly. We start at fCK/2,
	// the speed the console reaches in normal operation.
	//
	// The visible register is NOT changed to match: ckc_q still resets to 00H,
	// so software reading CKC sees FCPUS=000 (fCK/32) while the core runs at
	// fCK/2. That inconsistency is the price of the override. Leave it unless
	// something depends on reading back the boot clock, in which case fix the
	// STOP commit path rather than this parameter.
	parameter [2:0] CKC_RESET_FCPUS = 3'b100,
	parameter [7:0] P0C_RESET = 8'h00,
	// MAME resets all port-control bytes to 0, and the open boot ROM only
	// explicitly clears P0C before handing off to the external BIOS.
	parameter [7:0] P1C_RESET = 8'h00,
	parameter [7:0] P2C_RESET = 8'h00,
	parameter [7:0] P3C_RESET = 8'h00
)
(
	input wire clk_sys_i,
	input wire phi0_ce_i,
	input wire phi1_ce_i,
	input wire resetb_i,
	input wire stop_disable_i,
	input wire warm_boot_i,
	// FPGA helper for variable-latency external ROM backing (for example,
	// MiSTer SDRAM-backed cartridge reads). Real SM8521 timing remains fixed;
	// only slow ROM-class external reads consult this helper.
	input wire rom_read_ready_i,
	input wire [64:0] rtc_i,
	input wire nmib_i,
	input wire intb_i,
	// Game.com board-level Power wake helper. The front-panel Power key can
	// leave STOP or HALT without becoming a generic runtime INTB source.
	input wire power_stop_wake_i,
	input wire [2:0] m_i,
	input wire [7:0] d_din_i,
	output [20:0] a_o,
	output [7:0] d_dout_o,
	output d_oe_o,
	output mce0b_o,
	output mce1b_o,
	output ioe0b_o,
	output ioe1b_o,
	output rdb_o,
	output wrb_o,
	input wire [7:0] vd_din_i,
	output [12:0] va_o,
	output [7:0] vd_dout_o,
	output vd_oe_o,
	output vce0b_o,
	output vce1b_o,
	output vrdb_o,
	output vwrb_o,
	input wire [7:0] p0_din_i,
	output [7:0] p0_dout_o,
	output [7:0] p0_oe_o,
	output [7:0] p0_pullup_o,
	input wire [7:0] p1_din_i,
	output [7:0] p1_dout_o,
	output [7:0] p1_oe_o,
	output [7:0] p1_pullup_o,
	input wire [7:0] p2_din_i,
	output [7:0] p2_dout_o,
	output [7:0] p2_oe_o,
	output [7:0] p2_pullup_o,
	input wire [7:0] p3_din_i,
	output [7:0] p3_dout_o,
	output [7:0] p3_oe_o,
	output [7:0] p3_pullup_o,
	output [7:0] p3_latch_o,
	output pio_scan_update_o,
	input wire rxdb_i,
	input wire vr_i,
	output txdb_o,
	output [7:0] sound_o,
	output fr_o,
	output lp_o,
	output xc_o,
	output [3:0] xd_o,
	output yd_o,
	output display_page_o,
	output [1:0] display_palette_o,
	output display_normal_black_o,
	output display_hdot_200_o,
	output [1:0] display_vlines_o,
	output display_dma_active_o,
	output stopped_o,
	output doffb_o,
	output clk_o,
	input wire savestate_pause_req_i,
	output wire savestate_pause_ready_o,
	input wire savestate_active_i,
	input wire [11:0] savestate_addr_i,
	input wire savestate_rd_i,
	input wire savestate_wr_i,
	input wire [7:0] savestate_wdata_i,
	input wire cheat_clear_i,
	input wire [128:0] cheat_code_i,
	output wire [7:0] savestate_rdata_o
);

	localparam FLAG_C = 8'h80;
	localparam FLAG_Z = 8'h40;
	localparam FLAG_S = 8'h20;
	localparam FLAG_V = 8'h10;
	localparam FLAG_D = 8'h08;
	localparam FLAG_H = 8'h04;
	localparam FLAG_B = 8'h02;
	localparam FLAG_I = 8'h01;
	localparam [15:0] UART_TX_BIT_PHI0_CYCLES_W = UART_TX_BIT_PHI0_CYCLES[15:0];
	localparam [23:0] WDT_FC12_PHI0_CYCLES_W = WDT_FC12_PHI0_CYCLES[23:0];
	localparam [23:0] WDT_FX5_PHI0_CYCLES_W = WDT_FX5_PHI0_CYCLES[23:0];
	localparam [15:0] RTC_YEAR_ADDR = 16'h0124;
	localparam [15:0] RTC_MONTH_ADDR = 16'h0125;
	localparam [15:0] RTC_DAY_ADDR = 16'h0126;
	localparam [15:0] RTC_HOUR_ADDR = 16'h0127;
	localparam [15:0] RTC_MINUTE_ADDR = 16'h0128;
	localparam [15:0] RTC_SECOND_ADDR = 16'h0129;
	// Single-operand byte operations, shared by the direct (01H-0DH) and
	// indirect (1AH/1BH) encodings. See unary_alu.
	localparam [3:0] UN_NEG  = 4'd0;
	localparam [3:0] UN_COM  = 4'd1;
	localparam [3:0] UN_RR   = 4'd2;
	localparam [3:0] UN_RL   = 4'd3;
	localparam [3:0] UN_RRC  = 4'd4;
	localparam [3:0] UN_RLC  = 4'd5;
	localparam [3:0] UN_SRL  = 4'd6;
	localparam [3:0] UN_INC  = 4'd7;
	localparam [3:0] UN_DEC  = 4'd8;
	localparam [3:0] UN_SRA  = 4'd9;
	localparam [3:0] UN_SLL  = 4'd10;
	localparam [3:0] UN_DA   = 4'd11;
	localparam [3:0] UN_SWAP = 4'd12;

	localparam ST_RESET             = 6'd0;
	localparam ST_FETCH_SETUP       = 6'd1;
	localparam ST_FETCH_SAMPLE      = 6'd2;
	localparam ST_DECODE            = 6'd3;
	localparam ST_OP_READ8_SETUP    = 6'd4;
	localparam ST_OP_READ8_SAMPLE   = 6'd5;
	localparam ST_OP_READ16H_SETUP  = 6'd6;
	localparam ST_OP_READ16H_SAMPLE = 6'd7;
	localparam ST_OP_READ16L_SETUP  = 6'd8;
	localparam ST_OP_READ16L_SAMPLE = 6'd9;
	localparam ST_EXECUTE           = 6'd10;
	localparam ST_STALL             = 6'd11;
	localparam ST_INT_PUSH0_SETUP   = 6'd12;
	localparam ST_INT_PUSH0_SAMPLE  = 6'd13;
	localparam ST_INT_PUSH1_SETUP   = 6'd14;
	localparam ST_INT_PUSH1_SAMPLE  = 6'd15;
	localparam ST_INT_PUSH2_SETUP   = 6'd16;
	localparam ST_INT_PUSH2_SAMPLE  = 6'd17;
	localparam ST_INT_VECH_SETUP    = 6'd18;
	localparam ST_INT_VECH_SAMPLE   = 6'd19;
	localparam ST_INT_VECL_SETUP    = 6'd20;
	localparam ST_INT_VECL_SAMPLE   = 6'd21;
	localparam ST_RET_POP0_SETUP    = 6'd22;
	localparam ST_RET_POP0_SAMPLE   = 6'd23;
	localparam ST_RET_POP1_SETUP    = 6'd24;
	localparam ST_RET_POP1_SAMPLE   = 6'd25;
	localparam ST_RET_POP2_SETUP    = 6'd26;
	localparam ST_RET_POP2_SAMPLE   = 6'd27;
	localparam ST_OP_READ8_16H_SETUP  = 6'd28;
	localparam ST_OP_READ8_16H_SAMPLE = 6'd29;
	localparam ST_OP_READ8_16L_SETUP  = 6'd30;
	localparam ST_OP_READ8_16L_SAMPLE = 6'd31;
	localparam ST_MEM_READ_SETUP      = 6'd32;
	localparam ST_MEM_READ_SAMPLE     = 6'd33;
	localparam ST_MEM_WRITE_SETUP     = 6'd34;
	localparam ST_MEM_WRITE_SAMPLE    = 6'd35;
	localparam ST_MULDIV_STEP         = 6'd36;
	localparam ST_DMA_READ_SETUP      = 6'd37;
	localparam ST_DMA_READ_SAMPLE     = 6'd38;
	localparam ST_DMA_DEST_SETUP      = 6'd39;
	localparam ST_DMA_DEST_SAMPLE     = 6'd40;
	localparam ST_DMA_WRITE_SETUP     = 6'd41;
	localparam ST_DMA_WRITE_SAMPLE    = 6'd42;
	localparam ST_FETCH_WAIT          = 6'd43;
	localparam ST_OP_READ8_WAIT       = 6'd44;
	localparam ST_OP_READ16H_WAIT     = 6'd45;
	localparam ST_OP_READ16L_WAIT     = 6'd46;
	localparam ST_OP_READ8_16H_WAIT   = 6'd47;
	localparam ST_OP_READ8_16L_WAIT   = 6'd48;
	localparam ST_MEM_READ_WAIT       = 6'd49;
	localparam ST_DMA_WINDOW_SETUP    = 6'd50;
	localparam ST_DMA_WINDOW_SAMPLE   = 6'd51;
	localparam ST_DMA_PACE            = 6'd52;
	localparam ST_INT_VECH_WAIT       = 6'd53;
	localparam ST_INT_VECL_WAIT       = 6'd54;
	localparam ST_RET_POP0_WAIT       = 6'd55;
	localparam ST_RET_POP1_WAIT       = 6'd56;
	localparam ST_RET_POP2_WAIT       = 6'd57;
	localparam ST_DMA_SOURCE_HOLD     = 6'd58;

	localparam CLASS_NONE             = 6'd0;
	localparam CLASS_MOVI_REG         = 6'd1;
	localparam CLASS_MOVI_SFR         = 6'd2;
	localparam CLASS_MOV_REG_TO_FIXED = 6'd3;
	localparam CLASS_MOV_FIXED_TO_REG = 6'd4;
	localparam CLASS_MOVW_IMM         = 6'd5;
	localparam CLASS_BR               = 6'd6;
	localparam CLASS_JMP              = 6'd7;
	localparam CLASS_CLR_FIXED        = 6'd8;
	localparam CLASS_MOVPS0_IMM       = 6'd9;
	localparam CLASS_MOVI_FIXED       = 6'd10;
	localparam CLASS_MOVW_FIXED       = 6'd11;
	localparam CLASS_MEM_MOV8         = 6'd12;
	localparam CLASS_MEM_CMP8         = 6'd13;
	localparam CLASS_MEM_BYTE_ALU     = 6'd14;
	localparam CLASS_DBNZ             = 6'd15;
	localparam CLASS_MEM_STORE8       = 6'd16;
	localparam CLASS_FIXED_BYTE_OP    = 6'd17;
	localparam CLASS_CALL_ABS         = 6'd18;
	localparam CLASS_MEM_MOVW_LOAD    = 6'd19;
	localparam CLASS_MEM_MOVW_STORE   = 6'd20;
	localparam CLASS_MOVW_COMPACT     = 6'd21;
	localparam CLASS_JMP_INDIRECT     = 6'd22;
	localparam CLASS_CALL_INDIRECT    = 6'd23;
	localparam CLASS_MOVW_DIRECT      = 6'd24;
	localparam CLASS_DIRECT_IMM_OP    = 6'd25;
	localparam CLASS_WORD_RR_OP       = 6'd26;
	localparam CLASS_WORD_IMM_OP      = 6'd27;
	localparam CLASS_BIT_BRANCH       = 6'd28;
	localparam CLASS_BIT_MODIFY       = 6'd29;
	localparam CLASS_BMOV_BF          = 6'd30;
	localparam CLASS_BF_LOGIC         = 6'd31;
	localparam CLASS_INDIRECT_CMP     = 6'd32;
	localparam CLASS_INDIRECT_MOV     = 6'd33;
	localparam CLASS_MOVM_MASK        = 6'd34;
	localparam CLASS_MUL_RR           = 6'd35;
	localparam CLASS_MUL_IMM          = 6'd36;
	localparam CLASS_DIV_RR           = 6'd37;
	localparam CLASS_DIV_IMM          = 6'd38;
	localparam CLASS_DIRECT_UNARY     = 6'd39;
	localparam CLASS_COMPACT_BYTE_OP  = 6'd40;
	localparam CLASS_STACK_DIRECT     = 6'd41;
	localparam CLASS_INDIRECT_REG_OP  = 6'd42;
	localparam CLASS_RI_BIT_MODIFY    = 6'd43;
	localparam CLASS_BYTE_INDIRECT_OP = 6'd44;
	localparam CLASS_RI_BIT_BRANCH    = 6'd45;
	localparam CLASS_EXTS_DIRECT      = 6'd46;
	localparam CLASS_BTST_DIRECT      = 6'd47;
	localparam CLASS_CALS             = 6'd48;
	localparam CLASS_DM               = 6'd49;

	localparam AK_NONE = 3'd0;
	localparam AK_GP   = 3'd1;
	localparam AK_SFR  = 3'd2;
	localparam AK_IRAM = 3'd3;
	localparam AK_IROM = 3'd4;
	localparam AK_EXT  = 3'd5;
	localparam AK_VRAM = 3'd6;

	localparam IRQ_NONE = 4'd0;
	localparam IRQ_DMA  = 4'd1;
	localparam IRQ_TIM0 = 4'd2;
	localparam IRQ_EXT  = 4'd3;
	localparam IRQ_UART = 4'd4;
	localparam IRQ_LCDC = 4'd5;
	localparam IRQ_TIM1 = 4'd6;
	localparam IRQ_CLK  = 4'd7;
	localparam IRQ_PIO  = 4'd8;

	localparam [7:0] IE0_VALID_MASK = 8'hD9;
	localparam [7:0] IE1_VALID_MASK = 8'h54;
	localparam [7:0] IR0_VALID_MASK = 8'hD9;
	localparam [7:0] IR1_VALID_MASK = 8'h54;

	localparam ACT_NORMAL = 4'd0;
	localparam ACT_FETCH  = 4'd1;
	localparam ACT_ILL    = 4'd2;
	localparam ACT_HALT   = 4'd3;
	localparam ACT_STOP   = 4'd4;
	localparam ACT_RET    = 4'd5;
	localparam ACT_IRET   = 4'd6;
	localparam ACT_CLRC   = 4'd7;
	localparam ACT_COMC   = 4'd8;
	localparam ACT_SETC   = 4'd9;
	localparam ACT_EI     = 4'd10;
	localparam ACT_DI     = 4'd11;

	localparam TIMER_RUNS_IN_HALT = 1'b1;

	localparam [11:0] SS_SCALAR_BASE = 12'd0;
	localparam [11:0] SS_SCALAR_SIZE = 12'd256;
	localparam [11:0] SS_SOUND_BASE  = 12'd256;
	localparam [11:0] SS_SOUND_SIZE  = 12'd16;
	localparam [11:0] SS_TIMERS_BASE = 12'd272;
	localparam [11:0] SS_TIMERS_SIZE = 12'd16;
	localparam [11:0] SS_IRAM_BASE   = 12'd288;
	localparam [11:0] SS_IRAM_SIZE   = 12'd1024;
	localparam [11:0] SS_GP_BASE     = 12'd1312;
	localparam [11:0] SS_GP_SIZE     = 12'd512;
	localparam [7:0] SS_SG0W_BASE    = 8'd197;
	localparam [7:0] SS_SG1W_BASE    = 8'd213;

	reg [20:0] a_q;
	reg [7:0] d_dout_q;
	reg d_oe_q;
	reg mce0b_q;
	reg mce1b_q;
	reg ioe0b_q;
	reg ioe1b_q;
	reg rdb_q;
	reg wrb_q;

	reg [12:0] va_q;
	reg [7:0] vd_dout_q;
	reg vd_oe_q;
	reg vce0b_q;
	reg vce1b_q;
	reg vrdb_q;
	reg vwrb_q;
	reg clk_q;

	reg [15:0] pc_q;
	reg [15:0] sp_q;
	reg [7:0] ps0_q;
	reg [7:0] ps1_q;
	reg [7:0] ie0_q;
	reg [7:0] ie1_q;
	reg [7:0] ir0_q;
	reg [7:0] ir1_q;
	reg irq_dma_pending_q;
	reg irq_tim0_pending_q;
	reg irq_uart_pending_q;
	reg irq_lcdc_pending_q;
	reg irq_tim1_pending_q;
	reg irq_clk_pending_q;
	reg irq_pio_pending_q;
	reg [7:0] sys_q;
	reg [7:0] ckc_q;
	reg [2:0] cpu_clock_select_q;
	reg [3:0] cpu_prescale_q;
	reg cpu_phi1_pending_q;
	reg [24:0] cpu_subclock_accum_q;
	reg cpu_subclock_phase_q;
	reg [7:0] p0_q;
	reg [7:0] p1_q;
	reg [7:0] p2_q;
	reg [7:0] p3_q;
	reg pio_scan_update_q;
	reg [7:0] p0c_q;
	reg [7:0] p1c_q;
	reg [7:0] p2c_q;
	reg [7:0] p3c_q;
	reg [7:0] mmu0_q;
	reg [7:0] mmu1_q;
	reg [7:0] mmu2_q;
	reg [7:0] mmu3_q;
	reg [7:0] mmu4_q;
	reg [7:0] mmu1_resolved_q;
	reg [7:0] mmu2_resolved_q;
	reg [7:0] mmu3_resolved_q;
	reg [7:0] mmu4_resolved_q;
	reg [7:0] lcc_q;
	reg [7:0] lch_q;
	reg [7:0] lcv_q;
	reg [7:0] dmc_q;
	reg [7:0] dmx1_q;
	reg [7:0] dmy1_q;
	reg [7:0] dmdx_q;
	reg [7:0] dmdy_q;
	reg [7:0] dmx2_q;
	reg [7:0] dmy2_q;
	reg [7:0] dmpl_q;
	reg [7:0] dmbr_q;
	reg [7:0] dmvp_q;
	reg [7:0] urtt_q;
	reg [7:0] urtr_q;
	reg [7:0] urts_q;
	reg [7:0] urtc_q;
	reg [7:0] sgc_q;
	reg [7:0] sg0l_q;
	reg [7:0] sg1l_q;
	reg [7:0] sg2l_q;
	reg [7:0] sg0th_q;
	reg [7:0] sg1th_q;
	reg [7:0] sg2th_q;
	reg [15:0] sg0t_q;
	reg [15:0] sg1t_q;
	reg [15:0] sg2t_q;
	reg [7:0] sgda_q;
	reg sgda_write_strobe_q;
	reg sound_phi0_div_q;
	reg [7:0] sg0w_q [0:15];
	reg [7:0] sg1w_q [0:15];
	reg [7:0] tm0c_q;
	reg [7:0] tm0_reload_q;
	reg [7:0] tm1c_q;
	reg [7:0] tm1_reload_q;
	reg [1:0] tm0c_write_seq_q;
	reg [1:0] tm0_reload_write_seq_q;
	reg [1:0] tm1c_write_seq_q;
	reg [1:0] tm1_reload_write_seq_q;
	reg [1:0] clkt_write_seq_q;
	reg clkt_run_q;
	reg clkt_minute_q;
	reg rtc_toggle_prev_q;
	reg rtc_seen_toggle_q;
	reg rtc_host_init_done_q;
	reg [7:0] rtc_year_q;
	reg [7:0] rtc_month_q;
	reg [7:0] rtc_day_q;
	reg [7:0] rtc_hour_q;
	reg [7:0] rtc_minute_q;
	reg [7:0] rtc_second_q;
	reg uart_tx_active_q;
	reg [15:0] uart_tx_div_q;
	reg [11:0] uart_tx_shift_q;
	reg [3:0] uart_tx_bits_q;
	reg uart_rx_active_q;
	reg [15:0] uart_rx_div_q;
	reg [7:0] uart_rx_shift_q;
	reg [2:0] uart_rx_state_q;
	reg [3:0] uart_rx_bit_q;
	reg uart_rx_parity_q;
	reg uart_rx_pe_q;
	reg uart_rx_fe_q;
	reg uart_rx_paren_q;
	reg uart_rx_odd_q;
	reg uart_rx_stop2_q;
	reg rxdb_prev_q;
	reg txdb_q;
	reg [7:0] wdt_q;
	reg [7:0] wdtc_q;
	reg [23:0] wdt_div_q;

	reg [7:0] opcode_q;
	reg [7:0] operand0_q;
	reg [7:0] operand1_q;
	reg [7:0] mem_byte_q;
	reg [7:0] alu_res_q;
	reg [7:0] target_addr_q;
	reg [3:0] target_reg_q;
	reg [3:0] pair_base_q;
	reg [5:0] op_class_q;

	// Operand reads for the current beat, filled at the top of the CPU block.
	// Every direct address the execute stage can use is settled before the
	// beat starts, so the address decode runs once per port here instead of
	// once at every use.
	reg [7:0] iram_stored_v;
	reg [7:0] rd_op0_v;
	reg [7:0] rd_op0_hi_v;
	reg [7:0] rd_op1_addr_v;
	reg [7:0] rd_op1_v;
	reg [7:0] rd_op1_hi_v;
	reg [7:0] rd_tgt_v;
	reg [7:0] rd_tgt_hi_v;
	reg [7:0] rd_ind_v;
	reg [7:0] rmb_ptr_v;
	reg [7:0] rmb_index_v;
	reg [7:0] rmb_addr_v;
	reg [7:0] indirect_addr_v;
	reg [2:0] op_stage_q;
	reg [5:0] state_q;
	reg [5:0] return_state_q;
	reg [1:0] read_wait_q;
	reg [5:0] stall_count_q;
	reg [17:0] warmup_q;
	reg halted_q;
	reg stopped_q;
	reg mmu0_en_q;
	reg nmi_pending_q;
	reg wdt_pending_q;
	reg ext_pending_q;
	reg ill_pending_q;
	reg wdt_reset_pending_q;
	reg nmib_prev_q;
	reg intb_prev_q;
	reg power_halt_wake_seen_q;
	reg iret_q;
	reg irq_resume_defer_q;
	reg [15:0] vector_addr_q;
	reg warm_boot_sysflag_turnon_suppress_q;

	reg [2:0] access_kind_q;
	reg [15:0] access_addr_q;
	reg [7:0] access_wdata_q;
	reg [15:0] eff_addr_q;
	reg [23:0] lcdc_dma_div_q;
	reg [23:0] lcdc_scan_div_q;
	reg [5:0] lcdc_shift_q;
	reg [4:0] lcdc_hphase_q;
	reg [7:0] lcdc_line_q;
	reg lcdc_vblank_q;
	reg lcdc_vblank_prev_q;
	reg video_vblank_prev_q;
	reg lcdc_fr_q;
	reg lcdc_lp_q;
	reg lcdc_xc_q;
	reg [7:0] lcdc_scan_byte_q;
	reg [3:0] lcdc_xd_q;
	reg lcdc_yd_q;
	reg dma_active_q;
	reg [1:0] dma_mode_q;
	// HALT-time DMA launch snapshot. DMC[7] only arms DMA; the live DM*
	// registers are captured when HALT actually hands control to the blitter.
	reg [7:0] dma_arm_src_x_q;
	reg [7:0] dma_arm_dst_x_q;
	reg [7:0] dma_arm_line_count_q;
	reg [7:0] dma_ctl_q;
	reg [7:0] dma_dmpl_q;
	reg [7:0] dma_dmbr_q;
	reg [7:0] dma_dmvp_q;
	reg dma_hdot_200_q;
	reg [7:0] dma_src_byte_q;
	reg [7:0] dma_src_next_byte_q;
	reg dma_src_byte_valid_q;
	reg dma_src_next_valid_q;
	reg [2:0] dma_packet_pixels_q;
	reg [7:0] dma_packet_byte_q;
	reg [13:0] dma_src_addr_q;
	reg [13:0] dma_src_line_q;
	reg [1:0] dma_src_phase_q;
	reg [7:0] dma_src_x_q;
	reg [7:0] dma_src_y_q;
	reg [13:0] dma_dst_addr_q;
	reg [13:0] dma_dst_line_q;
	reg [7:0] dma_dst_x_q;
	reg [7:0] dma_dst_y_q;
	reg [7:0] dma_line_count_q;
	reg [7:0] dma_row_count_q;

	reg [9:0] ram_addr_q;
	reg ram_wren_q;
	reg [7:0] ram_wdata_q;
	reg savestate_pause_ready_q;
	reg savestate_clock_frozen_q;
	localparam [24:0] SUBCLOCK_RELOAD_THRESHOLD = CLK_SYS_HZ - SUBCLOCK_HZ;
	wire [7:0] ram_rdata_w;
	wire cpu_subclock_tick_w = cpu_subclock_accum_q >= SUBCLOCK_RELOAD_THRESHOLD;
	wire cpu_subclock_phi0_ce_w = cpu_subclock_tick_w && !savestate_clock_frozen_q &&
		!cpu_subclock_phase_q &&
		(cpu_clock_select_q == 3'b111);
	wire cpu_subclock_phi1_ce_w = cpu_subclock_tick_w && !savestate_clock_frozen_q &&
		cpu_subclock_phase_q &&
		(cpu_clock_select_q == 3'b111);
	wire cpu_main_phi0_ce_w = phi0_ce_i && (cpu_clock_select_q <= 3'b100) &&
		(cpu_prescale_q == 4'd0);
	wire cpu_core_phi0_ce_w = cpu_main_phi0_ce_w || cpu_subclock_phi0_ce_w;
	wire cpu_core_phi1_ce_w = cpu_phi1_pending_q &&
		(((cpu_clock_select_q == 3'b111) && cpu_subclock_phi1_ce_w) ||
		((cpu_clock_select_q != 3'b111) && phi1_ce_i));
	wire ss_scalar_sel_w = savestate_active_i &&
		(savestate_addr_i < (SS_SCALAR_BASE + SS_SCALAR_SIZE));
	wire ss_sound_sel_w = savestate_active_i &&
		(savestate_addr_i >= SS_SOUND_BASE) &&
		(savestate_addr_i < (SS_SOUND_BASE + SS_SOUND_SIZE));
	wire ss_timers_sel_w = savestate_active_i &&
		(savestate_addr_i >= SS_TIMERS_BASE) &&
		(savestate_addr_i < (SS_TIMERS_BASE + SS_TIMERS_SIZE));
	wire ss_iram_sel_w = savestate_active_i &&
		(savestate_addr_i >= SS_IRAM_BASE) &&
		(savestate_addr_i < (SS_IRAM_BASE + SS_IRAM_SIZE));
	wire ss_gp_sel_w = savestate_active_i &&
		(savestate_addr_i >= SS_GP_BASE) &&
		(savestate_addr_i < (SS_GP_BASE + SS_GP_SIZE));
	wire [9:0] ss_iram_addr_w = savestate_addr_i[9:0] - SS_IRAM_BASE[9:0];
	wire [8:0] ss_gp_addr_w = savestate_addr_i[8:0] - SS_GP_BASE[8:0];
	wire [4:0] ss_sound_addr_w = savestate_addr_i[4:0] - SS_SOUND_BASE[4:0];
	wire [4:0] ss_timers_addr_w = savestate_addr_i[4:0] - SS_TIMERS_BASE[4:0];
	wire ss_iram_direct_low_sel_w = ss_iram_sel_w && (ss_iram_addr_w[9:7] == 3'b001);
	wire [9:0] ram_addr_mux_w = ss_iram_sel_w ? ss_iram_addr_w : ram_addr_q;
	wire ram_wren_mux_w = ss_iram_sel_w ? savestate_wr_i : ram_wren_q;
	wire [7:0] ram_wdata_mux_w = ss_iram_sel_w ? savestate_wdata_i : ram_wdata_q;
	wire [7:0] gp_ss_rdata_w;
	wire [7:0] sound_ss_rdata_w;
	wire [7:0] timers_ss_rdata_w;

	reg [15:0] md_acc_q;
	reg [15:0] md_shift_q;
	reg [7:0] md_work_q;
	reg [8:0] md_rem_q;
	reg [4:0] md_count_q;
	// The pending bus accesses for this beat. A beat drives at most one read
	// and one write; normally only one of the two, but a DMA prefetch beat
	// starts the VRAM write and the external source read together.
	reg bus_wr_req_v;
	reg [15:0] bus_wr_addr_v;
	reg [7:0] bus_wr_data_v;

	reg bus_rd_req_v;
	reg [15:0] bus_rd_addr_v;
	reg [2:0] bus_rd_kind_v;
	reg bus_rd_kind_known_v;
	reg bus_rd_phys_v;
	reg [20:0] bus_rd_phys_addr_v;
	reg bus_rd_pick_state_v;
	reg [5:0] bus_rd_wait_state_v;
	reg [5:0] bus_rd_sample_state_v;

	// The LCDC scanline fetch that continues through idle bus beats.
	reg lcdc_scan_active_v;
	reg [12:0] lcdc_scan_addr_v;

	// The pending address-generation request for this beat. See request_addr.
	localparam [2:0] AGEN_NONE = 3'd0;
	localparam [2:0] AGEN_RMW  = 3'd1;
	localparam [2:0] AGEN_SMW  = 3'd2;
	localparam [2:0] AGEN_ARG2 = 3'd3;
	localparam [2:0] AGEN_RI   = 3'd4;

	reg [2:0] agen_kind_v;
	reg [7:0] agen_desc_v;
	reg [15:0] agen_imm_v;

	// Pending direct-address writes for this beat. Call sites queue here and
	// one drain at the end of the CPU block performs the accesses, so the SFR
	// write decode exists three times rather than at all seventy call sites.
	// Three slots is the widest beat there is: a 16-bit store plus the DIV
	// remainder.
	reg [1:0] dwr_count_v;
	reg [7:0] dwr_addr0_v;
	reg [7:0] dwr_data0_v;
	reg [7:0] dwr_addr1_v;
	reg [7:0] dwr_data1_v;
	reg [7:0] dwr_addr2_v;
	reg [7:0] dwr_data2_v;

	reg [11:0] rom_addr_q;
	wire [7:0] rom_rdata_w;

	wire [3:0] decode_action_w;
	wire [5:0] decode_op_class_w;
	wire [2:0] decode_op_stage_w;
	wire [5:0] decode_next_state_w;
	wire decode_target_reg_we_w;
	wire [3:0] decode_target_reg_w;
	wire decode_pair_base_we_w;
	wire [3:0] decode_pair_base_w;
	wire decode_target_addr_we_w;
	wire [7:0] decode_target_addr_w;

	wire gp_store_init_active_q;
	reg [7:0] gp_write_mask_a_q;
	reg [7:0] gp_write_mask_b_q;
	reg [63:0] gp_write_data_a_q;
	reg [63:0] gp_write_data_b_q;
	wire [127:0] gp_shadow_flat_w;
	reg [7:0] gp_shadow_q [0:15];
`ifndef SYNTHESIS
	reg [7:0] reg_bank_q [0:263];
	reg [7:0] sfr_shadow_q [0:127];
`endif
	reg [7:0] sfr_hole_18_q;
	reg [7:0] sfr_hole_1b_q;
	reg [7:0] sfr_hole_29_q;
	reg [7:0] sfr_hole_2a_q;
	reg [7:0] sfr_hole_2f_q;
	reg [7:0] sfr_hole_33_q;
	reg [7:0] sfr_hole_3e_q;
	reg [7:0] sfr_hole_3f_q;
	reg [7:0] sfr_hole_41_q;
	reg [7:0] sfr_hole_43_q;
	reg [7:0] sfr_hole_45_q;
	reg [7:0] sfr_hole_4b_q;
	reg [7:0] sfr_hole_4f_q;
	reg [7:0] sfr_hole_55_q;
	reg [7:0] sfr_hole_56_q;
	reg [7:0] sfr_hole_57_q;
	reg [7:0] sfr_hole_58_q;
	reg [7:0] sfr_hole_59_q;
	reg [7:0] sfr_hole_5a_q;
	reg [7:0] sfr_hole_5b_q;
	reg [7:0] sfr_hole_5c_q;
	reg [7:0] sfr_hole_5d_q;
	reg [7:0] iram_lo_shadow_q [0:127];
`ifndef SYNTHESIS
	reg [7:0] lowmem_q [0:255];
	integer reg_reset_i;
	integer sfr_shadow_reset_i;
	integer lowmem_reset_i;
`endif
	integer iram_lo_shadow_reset_i;
	integer sg_reset_i;

	assign a_o = a_q;
	assign d_dout_o = d_dout_q;
	assign d_oe_o = d_oe_q;
	assign mce0b_o = mce0b_q;
	assign mce1b_o = mce1b_q;
	assign ioe0b_o = ioe0b_q;
	assign ioe1b_o = ioe1b_q;
	assign rdb_o = rdb_q;
	assign wrb_o = wrb_q;
	assign va_o = va_q;
	assign vd_dout_o = vd_dout_q;
	assign vd_oe_o = vd_oe_q;
	assign vce0b_o = vce0b_q;
	assign vce1b_o = vce1b_q;
	assign vrdb_o = vrdb_q;
	assign vwrb_o = vwrb_q;
	assign p0_dout_o = p0_q;
	assign p1_dout_o = p1_q;
	assign p2_dout_o = p2_q;
	assign p0_oe_o = port_drive_mask(p0_q, p0c_q, 8'hFF);
	assign p1_oe_o = port_drive_mask(p1_q, p1c_q, 8'hFF);
	assign p2_oe_o = port_drive_mask(p2_q, p2c_q, 8'hFF);
	assign p0_pullup_o = port_pullup_mask(p0c_q);
	assign p1_pullup_o = port_pullup_mask(p1c_q);
	assign p2_pullup_o = port_pullup_mask(p2c_q);
	assign pio_scan_update_o = pio_scan_update_q;
	assign txdb_o = txdb_q;
	wire [127:0] sg0_wave_w = {
		sg0w_q[15], sg0w_q[14], sg0w_q[13], sg0w_q[12],
		sg0w_q[11], sg0w_q[10], sg0w_q[9], sg0w_q[8],
		sg0w_q[7], sg0w_q[6], sg0w_q[5], sg0w_q[4],
		sg0w_q[3], sg0w_q[2], sg0w_q[1], sg0w_q[0]
	};
	wire [127:0] sg1_wave_w = {
		sg1w_q[15], sg1w_q[14], sg1w_q[13], sg1w_q[12],
		sg1w_q[11], sg1w_q[10], sg1w_q[9], sg1w_q[8],
		sg1w_q[7], sg1w_q[6], sg1w_q[5], sg1w_q[4],
		sg1w_q[3], sg1w_q[2], sg1w_q[1], sg1w_q[0]
	};
	wire [7:0] sound_pcm_w;
	wire sound_core_reset_w = phi0_ce_i && wdt_reset_pending_q;
	wire timers_core_reset_w = phi0_ce_i && wdt_reset_pending_q;
	// Real SG2 captures line up the 32767-state noise pattern against the
	// native 5 MHz phi0 cadence. TM0/TM1 and SG now share that local tick.
	wire timers_stopped_w = stopped_q || (halted_q && !TIMER_RUNS_IN_HALT);
	wire [7:0] tm0d_q;
	wire [15:0] tm0_div_q;
	wire tm0_out_q;
	wire [7:0] tm1d_q;
	wire [15:0] tm1_div_q;
	wire tm1_out_q;
	// P3C[7:6] output modes expose the Timer 1 clock on P37.
	wire [7:0] p3_pin_dout_w = {tm1_out_q, p3_q[6:0]};
	wire [5:0] clkt_count_q;
	wire [23:0] clkt_div_q;
	wire tm0_irq_event_w;
	wire tm1_irq_event_w;
	wire clkt_second_event_w;
	wire clk_irq_event_w;

	assign sound_o = sound_pcm_w;
	assign p3_dout_o = p3_pin_dout_w;
	assign p3_oe_o = port_drive_mask(p3_pin_dout_w, p3c_q, 8'h3F);
	assign p3_pullup_o = port_pullup_mask(p3c_q);
	assign p3_latch_o = p3_q;
	assign fr_o = lcdc_fr_q;
	assign lp_o = lcdc_lp_q;
	assign xc_o = lcdc_xc_q;
	assign xd_o = lcdc_xd_q;
	assign yd_o = lcdc_yd_q;
	assign display_page_o = lcc_q[6];
	assign display_palette_o = lcc_q[5:4];
	assign display_normal_black_o = lcc_q[0];
	assign display_hdot_200_o = lch_q[5];
	assign display_vlines_o = lcv_q[5:4];
	assign display_dma_active_o = dma_active_q;
	assign stopped_o = stopped_q;
	assign doffb_o = lcc_q[7];
	assign clk_o = clk_q;
	assign savestate_pause_ready_o = savestate_pause_ready_q;
	assign savestate_rdata_o =
		ss_scalar_sel_w ? savestate_scalar_read(savestate_addr_i[7:0]) :
		ss_sound_sel_w ? sound_ss_rdata_w :
		ss_timers_sel_w ? timers_ss_rdata_w :
		ss_iram_direct_low_sel_w ? iram_lo_shadow_q[ss_iram_addr_w[6:0]] :
		ss_iram_sel_w ? ram_rdata_w :
		ss_gp_sel_w ? gp_ss_rdata_w :
		8'h00;

	//========================================================================
	// Ports and general-purpose registers
	//========================================================================
	function [7:0] port_output_select_mask;
		input [7:0] port_ctl;
		begin
			port_output_select_mask = {
				{2{port_ctl[7]}},
				{2{port_ctl[5]}},
				{2{port_ctl[3]}},
				{2{port_ctl[1]}}
			};
		end
	endfunction

	// od_capable marks which pins have an open-drain encoding at all. Every
	// P0C-P2C field offers one, but P3C bits 7:6 give the same
	// "output / Timer 1 clock on P37" for both 10 and 11, so P37 has no
	// open-drain mode and must keep driving high.
	function [7:0] port_drive_mask;
		input [7:0] port_dout;
		input [7:0] port_ctl;
		input [7:0] od_capable;
		reg [7:0] output_select_v;
		reg [7:0] open_drain_v;
		begin
			output_select_v = port_output_select_mask(port_ctl);
			open_drain_v = od_capable & {
				{2{&port_ctl[7:6]}},
				{2{&port_ctl[5:4]}},
				{2{&port_ctl[3:2]}},
				{2{&port_ctl[1:0]}}
			};
			// Open-drain outputs only actively drive low. When the port latch is
			// high, the pin must release so the external pull-up can source it.
			port_drive_mask = output_select_v & (~open_drain_v | ~port_dout);
		end
	endfunction

	function [7:0] port_pullup_mask;
		input [7:0] port_ctl;
		begin
			port_pullup_mask = {
				{2{~port_ctl[7] & port_ctl[6]}},
				{2{~port_ctl[5] & port_ctl[4]}},
				{2{~port_ctl[3] & port_ctl[2]}},
				{2{~port_ctl[1] & port_ctl[0]}}
			};
		end
	endfunction

	function [7:0] resolve_port_read;
		input [7:0] port_dout;
		input [7:0] port_ctl;
		input [7:0] port_din;
		reg [7:0] output_select_v;
		begin
			output_select_v = port_output_select_mask(port_ctl);
			resolve_port_read = (port_dout & output_select_v) | (port_din & ~output_select_v);
		end
	endfunction

	// Pair-select fields number their pairs R0,R8,R2,R10,R4,R12,R6,R14: the
	// low bit picks the upper eight registers and the other two pick the pair
	// within a half.
	function [3:0] pair_base;
		input [2:0] pair_sel;
		begin
			pair_base = {pair_sel[0], pair_sel[2:1], 1'b0};
		end
	endfunction

`ifndef SYNTHESIS
	function [8:0] reg_bank_index;
		input [3:0] reg_idx;
		reg [8:0] base_v;
		begin
			base_v = {1'b0, ps0_q[7:3], 3'b000};
			reg_bank_index = base_v + {5'b00000, reg_idx};
		end
	endfunction
`endif

	function [7:0] gp_read;
		input [3:0] reg_idx;
		begin
			// Keep the hot GP read path on the 16-byte active-bank mirror instead
			// of inferring a 264:1 combinational mux across the whole bank store.
			case (reg_idx)
				4'h0: gp_read = gp_shadow_q[0];
				4'h1: gp_read = gp_shadow_q[1];
				4'h2: gp_read = gp_shadow_q[2];
				4'h3: gp_read = gp_shadow_q[3];
				4'h4: gp_read = gp_shadow_q[4];
				4'h5: gp_read = gp_shadow_q[5];
				4'h6: gp_read = gp_shadow_q[6];
				4'h7: gp_read = gp_shadow_q[7];
				4'h8: gp_read = gp_shadow_q[8];
				4'h9: gp_read = gp_shadow_q[9];
				4'hA: gp_read = gp_shadow_q[10];
				4'hB: gp_read = gp_shadow_q[11];
				4'hC: gp_read = gp_shadow_q[12];
				4'hD: gp_read = gp_shadow_q[13];
				4'hE: gp_read = gp_shadow_q[14];
				default: gp_read = gp_shadow_q[15];
			endcase
		end
	endfunction

	task gp_write;
		input [3:0] reg_idx;
		input [7:0] data;
`ifndef SYNTHESIS
		reg [8:0] bank_idx_v;
`endif
		begin
			if (reg_idx[3]) begin
				gp_write_mask_b_q[reg_idx[2:0]] <= 1'b1;
				case (reg_idx[2:0])
					3'd0: gp_write_data_b_q[7:0] <= data;
					3'd1: gp_write_data_b_q[15:8] <= data;
					3'd2: gp_write_data_b_q[23:16] <= data;
					3'd3: gp_write_data_b_q[31:24] <= data;
					3'd4: gp_write_data_b_q[39:32] <= data;
					3'd5: gp_write_data_b_q[47:40] <= data;
					3'd6: gp_write_data_b_q[55:48] <= data;
					default: gp_write_data_b_q[63:56] <= data;
				endcase
			end else begin
				gp_write_mask_a_q[reg_idx[2:0]] <= 1'b1;
				case (reg_idx[2:0])
					3'd0: gp_write_data_a_q[7:0] <= data;
					3'd1: gp_write_data_a_q[15:8] <= data;
					3'd2: gp_write_data_a_q[23:16] <= data;
					3'd3: gp_write_data_a_q[31:24] <= data;
					3'd4: gp_write_data_a_q[39:32] <= data;
					3'd5: gp_write_data_a_q[47:40] <= data;
					3'd6: gp_write_data_a_q[55:48] <= data;
					default: gp_write_data_a_q[63:56] <= data;
				endcase
			end
`ifndef SYNTHESIS
			bank_idx_v = reg_bank_index(reg_idx);
			reg_bank_q[bank_idx_v[8:0]] <= data;
`endif
		end
	endtask

	// The low 128 bytes of IRAM are shadowed so reads can be combinational.
	// Bit 1 of the system flag at BCh is forced low on the first write after a
	// warm boot, so the BIOS does not see a stale power-on flag. The byte that
	// was actually stored is left in iram_stored_v.
	task write_iram_shadow;
		input [7:0] addr;
		input [7:0] data;
		begin
			iram_stored_v = data;
			if (warm_boot_sysflag_turnon_suppress_q && (addr == 8'hBC)) begin
				iram_stored_v = {data[7:2], 1'b0, data[0]};
				warm_boot_sysflag_turnon_suppress_q <= 1'b0;
			end
			iram_lo_shadow_q[addr[6:0]] <= iram_stored_v;
`ifndef SYNTHESIS
			lowmem_q[addr] <= iram_stored_v;
`endif
		end
	endtask

	// Six IRAM bytes mirror the host clock. Keep the mirrors in step with
	// whatever the program writes there.
	task write_rtc_mirror;
		input [15:0] addr;
		input [7:0] data;
		begin
			case (addr)
				RTC_YEAR_ADDR: rtc_year_q <= data;
				RTC_MONTH_ADDR: rtc_month_q <= data;
				RTC_DAY_ADDR: rtc_day_q <= data;
				RTC_HOUR_ADDR: rtc_hour_q <= data;
				RTC_MINUTE_ADDR: rtc_minute_q <= data;
				RTC_SECOND_ADDR: rtc_second_q <= data;
				default: begin
				end
			endcase
		end
	endtask

	task shadow_sfr_write;
		input [6:0] addr;
		input [7:0] data;
		begin
`ifndef SYNTHESIS
			sfr_shadow_q[addr] <= data;
`endif
		end
	endtask

`ifndef SYNTHESIS
	task init_sfr_shadow_reset;
		begin
			sfr_shadow_q[7'h10] <= 8'h00;
			sfr_shadow_q[7'h11] <= 8'h00;
			sfr_shadow_q[7'h12] <= 8'h00;
			sfr_shadow_q[7'h13] <= 8'h00;
			sfr_shadow_q[7'h14] <= 8'h00;
			sfr_shadow_q[7'h15] <= 8'h00;
			sfr_shadow_q[7'h16] <= 8'h00;
			sfr_shadow_q[7'h17] <= 8'h00;
			sfr_shadow_q[7'h19] <= 8'h00;
			sfr_shadow_q[7'h1A] <= 8'h00;
			sfr_shadow_q[7'h1C] <= 8'h00;
			sfr_shadow_q[7'h1D] <= 8'h00;
			sfr_shadow_q[7'h1E] <= 8'h00;
			sfr_shadow_q[7'h1F] <= 8'h00;
			sfr_shadow_q[7'h20] <= P0C_RESET;
			sfr_shadow_q[7'h21] <= P1C_RESET;
			sfr_shadow_q[7'h22] <= P2C_RESET;
			sfr_shadow_q[7'h23] <= P3C_RESET;
			sfr_shadow_q[7'h24] <= 8'h00;
			sfr_shadow_q[7'h25] <= 8'h00;
			sfr_shadow_q[7'h26] <= 8'h00;
			sfr_shadow_q[7'h27] <= 8'h00;
			sfr_shadow_q[7'h28] <= 8'h00;
			sfr_shadow_q[7'h2B] <= 8'hFF;
			sfr_shadow_q[7'h2C] <= 8'h00;
			sfr_shadow_q[7'h2D] <= 8'h02;
			sfr_shadow_q[7'h2E] <= 8'h00;
			sfr_shadow_q[7'h30] <= 8'hB0;
			sfr_shadow_q[7'h31] <= 8'h07;
			sfr_shadow_q[7'h32] <= 8'h27;
			sfr_shadow_q[7'h34] <= 8'h00;
			sfr_shadow_q[7'h35] <= 8'h00;
			sfr_shadow_q[7'h36] <= 8'h00;
			sfr_shadow_q[7'h37] <= 8'h00;
			sfr_shadow_q[7'h38] <= 8'h00;
			sfr_shadow_q[7'h39] <= 8'h00;
			sfr_shadow_q[7'h3A] <= 8'h00;
			sfr_shadow_q[7'h3B] <= 8'h00;
			sfr_shadow_q[7'h3C] <= 8'h00;
			sfr_shadow_q[7'h3D] <= 8'h00;
			sfr_shadow_q[7'h40] <= 8'h00;
			sfr_shadow_q[7'h42] <= 8'h00;
			sfr_shadow_q[7'h44] <= 8'h00;
			sfr_shadow_q[7'h46] <= 8'h00;
			sfr_shadow_q[7'h47] <= 8'h00;
			sfr_shadow_q[7'h48] <= 8'h00;
			sfr_shadow_q[7'h49] <= 8'h00;
			sfr_shadow_q[7'h4A] <= 8'h00;
			sfr_shadow_q[7'h4C] <= 8'h00;
			sfr_shadow_q[7'h4D] <= 8'h00;
			sfr_shadow_q[7'h4E] <= 8'h00;
			sfr_shadow_q[7'h50] <= 8'h00;
			sfr_shadow_q[7'h51] <= 8'h00;
			sfr_shadow_q[7'h52] <= 8'h00;
			sfr_shadow_q[7'h53] <= 8'h00;
			sfr_shadow_q[7'h54] <= 8'h00;
			sfr_shadow_q[7'h5E] <= 8'h00;
			sfr_shadow_q[7'h5F] <= 8'h38;
			lowmem_q[8'h10] <= 8'h00;
			lowmem_q[8'h11] <= 8'h00;
			lowmem_q[8'h12] <= 8'h00;
			lowmem_q[8'h13] <= 8'h00;
			lowmem_q[8'h14] <= 8'h00;
			lowmem_q[8'h15] <= 8'h00;
			lowmem_q[8'h16] <= 8'h00;
			lowmem_q[8'h17] <= 8'h00;
			lowmem_q[8'h19] <= 8'h00;
			lowmem_q[8'h1A] <= 8'h00;
			lowmem_q[8'h1C] <= 8'h00;
			lowmem_q[8'h1D] <= 8'h00;
			lowmem_q[8'h1E] <= 8'h00;
			lowmem_q[8'h1F] <= 8'h00;
			lowmem_q[8'h20] <= P0C_RESET;
			lowmem_q[8'h21] <= P1C_RESET;
			lowmem_q[8'h22] <= P2C_RESET;
			lowmem_q[8'h23] <= P3C_RESET;
			lowmem_q[8'h24] <= 8'h00;
			lowmem_q[8'h25] <= 8'h00;
			lowmem_q[8'h26] <= 8'h00;
			lowmem_q[8'h27] <= 8'h00;
			lowmem_q[8'h28] <= 8'h00;
			lowmem_q[8'h2B] <= 8'hFF;
			lowmem_q[8'h2C] <= 8'h00;
			lowmem_q[8'h2D] <= 8'h02;
			lowmem_q[8'h2E] <= 8'h00;
			lowmem_q[8'h30] <= 8'hB0;
			lowmem_q[8'h31] <= 8'h07;
			lowmem_q[8'h32] <= 8'h27;
			lowmem_q[8'h34] <= 8'h00;
			lowmem_q[8'h35] <= 8'h00;
			lowmem_q[8'h36] <= 8'h00;
			lowmem_q[8'h37] <= 8'h00;
			lowmem_q[8'h38] <= 8'h00;
			lowmem_q[8'h39] <= 8'h00;
			lowmem_q[8'h3A] <= 8'h00;
			lowmem_q[8'h3B] <= 8'h00;
			lowmem_q[8'h3C] <= 8'h00;
			lowmem_q[8'h3D] <= 8'h00;
			lowmem_q[8'h40] <= 8'h00;
			lowmem_q[8'h42] <= 8'h00;
			lowmem_q[8'h44] <= 8'h00;
			lowmem_q[8'h46] <= 8'h00;
			lowmem_q[8'h47] <= 8'h00;
			lowmem_q[8'h48] <= 8'h00;
			lowmem_q[8'h49] <= 8'h00;
			lowmem_q[8'h4A] <= 8'h00;
			lowmem_q[8'h4C] <= 8'h00;
			lowmem_q[8'h4D] <= 8'h00;
			lowmem_q[8'h4E] <= 8'h00;
			lowmem_q[8'h50] <= 8'h00;
			lowmem_q[8'h51] <= 8'h00;
			lowmem_q[8'h52] <= 8'h00;
			lowmem_q[8'h53] <= 8'h00;
			lowmem_q[8'h54] <= 8'h00;
			lowmem_q[8'h5E] <= 8'h00;
			lowmem_q[8'h5F] <= 8'h38;
		end
	endtask
`endif

	task reset_sfr_holes;
		begin
			sfr_hole_18_q <= 8'h00;
			sfr_hole_1b_q <= 8'h00;
			sfr_hole_29_q <= 8'h00;
			sfr_hole_2a_q <= 8'h00;
			sfr_hole_2f_q <= 8'h00;
			sfr_hole_33_q <= 8'h00;
			sfr_hole_3e_q <= 8'h00;
			sfr_hole_3f_q <= 8'h00;
			sfr_hole_41_q <= 8'h00;
			sfr_hole_43_q <= 8'h00;
			sfr_hole_45_q <= 8'h00;
			sfr_hole_4b_q <= 8'h00;
			sfr_hole_4f_q <= 8'h00;
			sfr_hole_55_q <= 8'h00;
			sfr_hole_56_q <= 8'h00;
			sfr_hole_57_q <= 8'h00;
			sfr_hole_58_q <= 8'h00;
			sfr_hole_59_q <= 8'h00;
			sfr_hole_5a_q <= 8'h00;
			sfr_hole_5b_q <= 8'h00;
			sfr_hole_5c_q <= 8'h00;
			sfr_hole_5d_q <= 8'h00;
		end
	endtask

	task write_sfr_hole;
		input [7:0] addr;
		input [7:0] data;
		begin
			case (addr)
				8'h18: sfr_hole_18_q <= data;
				8'h1B: sfr_hole_1b_q <= data;
				8'h29: sfr_hole_29_q <= data;
				8'h2A: sfr_hole_2a_q <= data;
				8'h2F: sfr_hole_2f_q <= data;
				8'h33: sfr_hole_33_q <= data;
				8'h3E: sfr_hole_3e_q <= data;
				8'h3F: sfr_hole_3f_q <= data;
				8'h41: sfr_hole_41_q <= data;
				8'h43: sfr_hole_43_q <= data;
				8'h45: sfr_hole_45_q <= data;
				8'h4B: sfr_hole_4b_q <= data;
				8'h4F: sfr_hole_4f_q <= data;
				8'h55: sfr_hole_55_q <= data;
				8'h56: sfr_hole_56_q <= data;
				8'h57: sfr_hole_57_q <= data;
				8'h58: sfr_hole_58_q <= data;
				8'h59: sfr_hole_59_q <= data;
				8'h5A: sfr_hole_5a_q <= data;
				8'h5B: sfr_hole_5b_q <= data;
				8'h5C: sfr_hole_5c_q <= data;
				8'h5D: sfr_hole_5d_q <= data;
				default: begin
				end
			endcase
		end
	endtask

	//========================================================================
	// Savestate scalars
	//========================================================================
	function [7:0] savestate_scalar_read;
		input [7:0] addr;
		reg [3:0] wave_idx_v;
		begin
			savestate_scalar_read = 8'h00;
			if ((addr >= SS_SG0W_BASE) && (addr < SS_SG1W_BASE)) begin
				wave_idx_v = addr[3:0] - SS_SG0W_BASE[3:0];
				savestate_scalar_read = sg0w_q[wave_idx_v];
			end else if ((addr >= SS_SG1W_BASE) && (addr < (SS_SG1W_BASE + 8'd16))) begin
				wave_idx_v = addr[3:0] - SS_SG1W_BASE[3:0];
				savestate_scalar_read = sg1w_q[wave_idx_v];
			end else begin
				case (addr)
					8'd0: savestate_scalar_read = pc_q[7:0];
					8'd1: savestate_scalar_read = pc_q[15:8];
					8'd2: savestate_scalar_read = sp_q[7:0];
					8'd3: savestate_scalar_read = sp_q[15:8];
					8'd4: savestate_scalar_read = ps0_q;
					8'd5: savestate_scalar_read = ps1_q;
					8'd6: savestate_scalar_read = ie0_q;
					8'd7: savestate_scalar_read = ie1_q;
					8'd8: savestate_scalar_read = ir0_q;
					8'd9: savestate_scalar_read = ir1_q;
					8'd10: savestate_scalar_read = {
						nmi_pending_q,
						irq_pio_pending_q,
						irq_clk_pending_q,
						irq_tim1_pending_q,
						irq_lcdc_pending_q,
						irq_uart_pending_q,
						irq_tim0_pending_q,
						irq_dma_pending_q
					};
					8'd11: savestate_scalar_read = {
						iret_q,
						mmu0_en_q,
						stopped_q,
						halted_q,
						wdt_reset_pending_q,
						ill_pending_q,
						ext_pending_q,
						wdt_pending_q
					};
					8'd12: savestate_scalar_read = sys_q;
					8'd13: savestate_scalar_read = ckc_q;
					8'd14: savestate_scalar_read = p0_q;
					8'd15: savestate_scalar_read = p1_q;
					8'd16: savestate_scalar_read = p2_q;
					8'd17: savestate_scalar_read = p3_q;
					8'd18: savestate_scalar_read = p0c_q;
					8'd19: savestate_scalar_read = p1c_q;
					8'd20: savestate_scalar_read = p2c_q;
					8'd21: savestate_scalar_read = p3c_q;
					8'd22: savestate_scalar_read = mmu0_q;
					8'd23: savestate_scalar_read = mmu1_q;
					8'd24: savestate_scalar_read = mmu2_q;
					8'd25: savestate_scalar_read = mmu3_q;
					8'd26: savestate_scalar_read = mmu4_q;
					8'd27: savestate_scalar_read = mmu1_resolved_q;
					8'd28: savestate_scalar_read = mmu2_resolved_q;
					8'd29: savestate_scalar_read = mmu3_resolved_q;
					8'd30: savestate_scalar_read = mmu4_resolved_q;
					8'd31: savestate_scalar_read = lcc_q;
					8'd32: savestate_scalar_read = lch_q;
					8'd33: savestate_scalar_read = lcv_q;
					8'd34: savestate_scalar_read = dmc_q;
					8'd35: savestate_scalar_read = dmx1_q;
					8'd36: savestate_scalar_read = dmy1_q;
					8'd37: savestate_scalar_read = dmdx_q;
					8'd38: savestate_scalar_read = dmdy_q;
					8'd39: savestate_scalar_read = dmx2_q;
					8'd40: savestate_scalar_read = dmy2_q;
					8'd41: savestate_scalar_read = dmpl_q;
					8'd42: savestate_scalar_read = dmbr_q;
					8'd43: savestate_scalar_read = dmvp_q;
					8'd44: savestate_scalar_read = urtt_q;
					8'd45: savestate_scalar_read = urtr_q;
					8'd46: savestate_scalar_read = urts_q;
					8'd47: savestate_scalar_read = urtc_q;
					8'd48: savestate_scalar_read = sgc_q;
					8'd49: savestate_scalar_read = sg0l_q;
					8'd50: savestate_scalar_read = sg1l_q;
					8'd51: savestate_scalar_read = sg2l_q;
					8'd52: savestate_scalar_read = sg0th_q;
					8'd53: savestate_scalar_read = sg1th_q;
					8'd54: savestate_scalar_read = sg2th_q;
					8'd55: savestate_scalar_read = sg0t_q[7:0];
					8'd56: savestate_scalar_read = sg0t_q[15:8];
					8'd57: savestate_scalar_read = sg1t_q[7:0];
					8'd58: savestate_scalar_read = sg1t_q[15:8];
					8'd59: savestate_scalar_read = sg2t_q[7:0];
					8'd60: savestate_scalar_read = sg2t_q[15:8];
					8'd61: savestate_scalar_read = sgda_q;
					8'd62: savestate_scalar_read = {7'b0000000, sound_phi0_div_q};
					8'd63: savestate_scalar_read = tm0c_q;
					8'd64: savestate_scalar_read = tm0_reload_q;
					8'd65: savestate_scalar_read = tm1c_q;
					8'd66: savestate_scalar_read = tm1_reload_q;
					8'd67: savestate_scalar_read = {
						clkt_write_seq_q,
						tm1_reload_write_seq_q,
						tm1c_write_seq_q,
						tm0_reload_write_seq_q[1:0]
					};
					8'd68: savestate_scalar_read = {4'b0000, clkt_minute_q, clkt_run_q, tm0c_write_seq_q};
					8'd69: savestate_scalar_read = {5'b00000, rtc_host_init_done_q, rtc_seen_toggle_q, rtc_toggle_prev_q};
					8'd70: savestate_scalar_read = rtc_year_q;
					8'd71: savestate_scalar_read = rtc_month_q;
					8'd72: savestate_scalar_read = rtc_day_q;
					8'd73: savestate_scalar_read = rtc_hour_q;
					8'd74: savestate_scalar_read = rtc_minute_q;
					8'd75: savestate_scalar_read = rtc_second_q;
					8'd76: savestate_scalar_read = {7'b0000000, uart_tx_active_q};
					8'd77: savestate_scalar_read = uart_tx_div_q[7:0];
					8'd78: savestate_scalar_read = uart_tx_div_q[15:8];
					8'd79: savestate_scalar_read = uart_tx_shift_q[7:0];
					8'd80: savestate_scalar_read = {4'b0000, uart_tx_shift_q[11:8]};
					8'd81: savestate_scalar_read = {4'b0000, uart_tx_bits_q};
					8'd82: savestate_scalar_read = {7'b0000000, uart_rx_active_q};
					8'd83: savestate_scalar_read = uart_rx_div_q[7:0];
					8'd84: savestate_scalar_read = uart_rx_div_q[15:8];
					8'd85: savestate_scalar_read = uart_rx_shift_q;
					8'd86: savestate_scalar_read = {5'b00000, uart_rx_state_q};
					8'd87: savestate_scalar_read = {4'b0000, uart_rx_bit_q};
					8'd88: savestate_scalar_read = {
						txdb_q,
						rxdb_prev_q,
						uart_rx_stop2_q,
						uart_rx_odd_q,
						uart_rx_paren_q,
						uart_rx_fe_q,
						uart_rx_pe_q,
						uart_rx_parity_q
					};
					8'd89: savestate_scalar_read = wdt_q;
					8'd90: savestate_scalar_read = wdtc_q;
					8'd91: savestate_scalar_read = wdt_div_q[7:0];
					8'd92: savestate_scalar_read = wdt_div_q[15:8];
					8'd93: savestate_scalar_read = wdt_div_q[23:16];
					8'd94: savestate_scalar_read = opcode_q;
					8'd95: savestate_scalar_read = operand0_q;
					8'd96: savestate_scalar_read = operand1_q;
					8'd97: savestate_scalar_read = mem_byte_q;
					8'd98: savestate_scalar_read = alu_res_q;
					8'd99: savestate_scalar_read = target_addr_q;
					8'd100: savestate_scalar_read = {target_reg_q, pair_base_q};
					8'd101: savestate_scalar_read = {2'b00, op_class_q};
					8'd102: savestate_scalar_read = {5'b00000, op_stage_q};
					8'd103: savestate_scalar_read = {2'b00, state_q};
					8'd104: savestate_scalar_read = {2'b00, return_state_q};
					8'd105: savestate_scalar_read = {stall_count_q, read_wait_q};
					8'd106: savestate_scalar_read = warmup_q[7:0];
					8'd107: savestate_scalar_read = warmup_q[15:8];
					8'd108: savestate_scalar_read = {6'b000000, warmup_q[17:16]};
					8'd109: savestate_scalar_read = vector_addr_q[7:0];
					8'd110: savestate_scalar_read = vector_addr_q[15:8];
					8'd111: savestate_scalar_read = {5'b00000, access_kind_q};
					8'd112: savestate_scalar_read = access_addr_q[7:0];
					8'd113: savestate_scalar_read = access_addr_q[15:8];
					8'd114: savestate_scalar_read = access_wdata_q;
					8'd115: savestate_scalar_read = eff_addr_q[7:0];
					8'd116: savestate_scalar_read = eff_addr_q[15:8];
					8'd117: savestate_scalar_read = md_acc_q[7:0];
					8'd118: savestate_scalar_read = md_acc_q[15:8];
					8'd119: savestate_scalar_read = md_shift_q[7:0];
					8'd120: savestate_scalar_read = md_shift_q[15:8];
					8'd121: savestate_scalar_read = md_work_q;
					8'd122: savestate_scalar_read = md_rem_q[7:0];
					8'd123: savestate_scalar_read = {7'b0000000, md_rem_q[8]};
					8'd124: savestate_scalar_read = {3'b000, md_count_q};
					8'd125: savestate_scalar_read = ram_addr_q[7:0];
					8'd126: savestate_scalar_read = {6'b000000, ram_addr_q[9:8]};
					8'd127: savestate_scalar_read = ram_wdata_q;
					8'd128: savestate_scalar_read = rom_addr_q[7:0];
					8'd129: savestate_scalar_read = {4'b0000, rom_addr_q[11:8]};
					8'd130: savestate_scalar_read = {2'b00, power_halt_wake_seen_q, irq_resume_defer_q, intb_prev_q, nmib_prev_q, warm_boot_sysflag_turnon_suppress_q, clk_q};
					8'd131: savestate_scalar_read = lcdc_dma_div_q[7:0];
					8'd132: savestate_scalar_read = lcdc_dma_div_q[15:8];
					8'd133: savestate_scalar_read = lcdc_dma_div_q[23:16];
					8'd134: savestate_scalar_read = lcdc_scan_div_q[7:0];
					8'd135: savestate_scalar_read = lcdc_scan_div_q[15:8];
					8'd136: savestate_scalar_read = lcdc_scan_div_q[23:16];
					8'd137: savestate_scalar_read = {2'b00, lcdc_shift_q};
					8'd138: savestate_scalar_read = {3'b000, lcdc_hphase_q};
					8'd139: savestate_scalar_read = lcdc_line_q;
					8'd140: savestate_scalar_read = {1'b0, video_vblank_prev_q, lcdc_yd_q, lcdc_xc_q, lcdc_lp_q, lcdc_fr_q, lcdc_vblank_prev_q, lcdc_vblank_q};
					8'd141: savestate_scalar_read = lcdc_scan_byte_q;
					8'd142: savestate_scalar_read = {4'b0000, lcdc_xd_q};
					8'd143: savestate_scalar_read = {dma_src_phase_q, 1'b0, dma_hdot_200_q, 1'b0, 2'b00, dma_active_q};
					8'd148: savestate_scalar_read = dma_arm_src_x_q;
					8'd150: savestate_scalar_read = dma_arm_dst_x_q;
					8'd152: savestate_scalar_read = dma_arm_line_count_q;
					8'd154: savestate_scalar_read = dma_ctl_q;
					8'd155: savestate_scalar_read = dma_dmpl_q;
					8'd156: savestate_scalar_read = dma_dmbr_q;
					8'd157: savestate_scalar_read = dma_dmvp_q;
					8'd158: savestate_scalar_read = dma_src_byte_q;
					8'd159: savestate_scalar_read = 8'h00;
					8'd160: savestate_scalar_read = dma_src_addr_q[7:0];
					8'd161: savestate_scalar_read = {2'b00, dma_src_addr_q[13:8]};
					8'd162: savestate_scalar_read = dma_src_line_q[7:0];
					8'd163: savestate_scalar_read = {2'b00, dma_src_line_q[13:8]};
					8'd164: savestate_scalar_read = dma_src_x_q;
					8'd165: savestate_scalar_read = dma_src_y_q;
					8'd166: savestate_scalar_read = dma_dst_addr_q[7:0];
					8'd167: savestate_scalar_read = {2'b00, dma_dst_addr_q[13:8]};
					8'd168: savestate_scalar_read = dma_dst_line_q[7:0];
					8'd169: savestate_scalar_read = {2'b00, dma_dst_line_q[13:8]};
					8'd170: savestate_scalar_read = dma_dst_x_q;
					8'd171: savestate_scalar_read = dma_dst_y_q;
					8'd172: savestate_scalar_read = dma_line_count_q;
					8'd173: savestate_scalar_read = dma_row_count_q;
					8'd174: savestate_scalar_read = {6'b000000, dma_mode_q};
					8'd175: savestate_scalar_read = sfr_hole_18_q;
					8'd176: savestate_scalar_read = sfr_hole_1b_q;
					8'd177: savestate_scalar_read = sfr_hole_29_q;
					8'd178: savestate_scalar_read = sfr_hole_2a_q;
					8'd179: savestate_scalar_read = sfr_hole_2f_q;
					8'd180: savestate_scalar_read = sfr_hole_33_q;
					8'd181: savestate_scalar_read = sfr_hole_3e_q;
					8'd182: savestate_scalar_read = sfr_hole_3f_q;
					8'd183: savestate_scalar_read = sfr_hole_41_q;
					8'd184: savestate_scalar_read = sfr_hole_43_q;
					8'd185: savestate_scalar_read = sfr_hole_45_q;
					8'd186: savestate_scalar_read = sfr_hole_4b_q;
					8'd187: savestate_scalar_read = sfr_hole_4f_q;
					8'd188: savestate_scalar_read = sfr_hole_55_q;
					8'd189: savestate_scalar_read = sfr_hole_56_q;
					8'd190: savestate_scalar_read = sfr_hole_57_q;
					8'd191: savestate_scalar_read = sfr_hole_58_q;
					8'd192: savestate_scalar_read = sfr_hole_59_q;
					8'd193: savestate_scalar_read = sfr_hole_5a_q;
					8'd194: savestate_scalar_read = sfr_hole_5b_q;
					8'd195: savestate_scalar_read = sfr_hole_5c_q;
					8'd196: savestate_scalar_read = sfr_hole_5d_q;
					8'd229: savestate_scalar_read = a_q[7:0];
					8'd230: savestate_scalar_read = a_q[15:8];
					8'd231: savestate_scalar_read = {3'b000, a_q[20:16]};
					8'd232: savestate_scalar_read = d_dout_q;
					8'd233: savestate_scalar_read = {mce0b_q, mce1b_q, ioe0b_q, ioe1b_q, rdb_q, wrb_q, d_oe_q, vd_oe_q};
					8'd234: savestate_scalar_read = va_q[7:0];
					8'd235: savestate_scalar_read = {3'b000, va_q[12:8]};
					8'd236: savestate_scalar_read = vd_dout_q;
					8'd237: savestate_scalar_read = {4'b0000, vwrb_q, vrdb_q, vce1b_q, vce0b_q};
					8'd238: savestate_scalar_read = {cpu_clock_select_encode(cpu_clock_select_q), cpu_prescale_q, cpu_phi1_pending_q};
					8'd239: savestate_scalar_read = cpu_subclock_accum_q[7:0];
					8'd240: savestate_scalar_read = cpu_subclock_accum_q[15:8];
					8'd241: savestate_scalar_read = cpu_subclock_accum_q[23:16];
					8'd242: savestate_scalar_read = {6'b000000, cpu_subclock_phase_q, cpu_subclock_accum_q[24]};
					// 197-199 would land inside the SG0 wave-RAM window above and
					// never reach this case, so the DMA pipeline lives at 243-245.
					8'd243: savestate_scalar_read = dma_src_next_byte_q;
					8'd244: savestate_scalar_read = dma_packet_byte_q;
					8'd245: savestate_scalar_read = {3'b000, dma_packet_pixels_q, dma_src_next_valid_q, dma_src_byte_valid_q};
					default: savestate_scalar_read = 8'h00;
				endcase
			end
		end
	endfunction

	task savestate_scalar_write;
		input [7:0] addr;
		input [7:0] data;
		reg [3:0] wave_idx_v;
		begin
			if ((addr >= SS_SG0W_BASE) && (addr < SS_SG1W_BASE)) begin
				wave_idx_v = addr[3:0] - SS_SG0W_BASE[3:0];
				sg0w_q[wave_idx_v] <= data;
			end else if ((addr >= SS_SG1W_BASE) && (addr < (SS_SG1W_BASE + 8'd16))) begin
				wave_idx_v = addr[3:0] - SS_SG1W_BASE[3:0];
				sg1w_q[wave_idx_v] <= data;
			end else begin
				case (addr)
					8'd0: pc_q[7:0] <= data;
					8'd1: pc_q[15:8] <= data;
					8'd2: sp_q[7:0] <= data;
					8'd3: sp_q[15:8] <= data;
					8'd4: ps0_q <= data;
					8'd5: ps1_q <= data;
					8'd6: ie0_q <= data & IE0_VALID_MASK;
					8'd7: ie1_q <= data & IE1_VALID_MASK;
					8'd8: ir0_q <= data & IR0_VALID_MASK;
					8'd9: ir1_q <= data & IR1_VALID_MASK;
					8'd10: begin
						irq_dma_pending_q <= data[0];
						irq_tim0_pending_q <= data[1];
						irq_uart_pending_q <= data[2];
						irq_lcdc_pending_q <= data[3];
						irq_tim1_pending_q <= data[4];
						irq_clk_pending_q <= data[5];
						irq_pio_pending_q <= data[6];
						nmi_pending_q <= data[7];
					end
					8'd11: begin
						wdt_pending_q <= data[0];
						ext_pending_q <= data[1];
						ill_pending_q <= data[2];
						wdt_reset_pending_q <= data[3];
						halted_q <= data[4];
						stopped_q <= data[5];
						mmu0_en_q <= data[6];
						iret_q <= data[7];
					end
					8'd12: sys_q <= data;
					8'd13: ckc_q <= data;
					8'd14: p0_q <= data;
					8'd15: p1_q <= data;
					8'd16: p2_q <= data;
					8'd17: p3_q <= data;
					8'd18: p0c_q <= data;
					8'd19: p1c_q <= data;
					8'd20: p2c_q <= data;
					8'd21: p3c_q <= data;
					8'd22: mmu0_q <= data;
					8'd23: mmu1_q <= data;
					8'd24: mmu2_q <= data;
					8'd25: mmu3_q <= data;
					8'd26: mmu4_q <= data;
					8'd27: mmu1_resolved_q <= data;
					8'd28: mmu2_resolved_q <= data;
					8'd29: mmu3_resolved_q <= data;
					8'd30: mmu4_resolved_q <= data;
					8'd31: lcc_q <= data;
					8'd32: lch_q <= data;
					8'd33: lcv_q <= data;
					8'd34: dmc_q <= data;
					8'd35: dmx1_q <= data;
					8'd36: dmy1_q <= data;
					8'd37: dmdx_q <= data;
					8'd38: dmdy_q <= data;
					8'd39: dmx2_q <= data;
					8'd40: dmy2_q <= data;
					8'd41: dmpl_q <= data;
					8'd42: dmbr_q <= data;
					8'd43: dmvp_q <= data;
					8'd44: urtt_q <= data;
					8'd45: urtr_q <= data;
					8'd46: urts_q <= data;
					8'd47: urtc_q <= data;
					8'd48: sgc_q <= data;
					8'd49: sg0l_q <= data;
					8'd50: sg1l_q <= data;
					8'd51: sg2l_q <= data;
					8'd52: sg0th_q <= data;
					8'd53: sg1th_q <= data;
					8'd54: sg2th_q <= data;
					8'd55: sg0t_q[7:0] <= data;
					8'd56: sg0t_q[15:8] <= data;
					8'd57: sg1t_q[7:0] <= data;
					8'd58: sg1t_q[15:8] <= data;
					8'd59: sg2t_q[7:0] <= data;
					8'd60: sg2t_q[15:8] <= data;
					8'd61: sgda_q <= data;
					8'd62: sound_phi0_div_q <= data[0];
					8'd63: tm0c_q <= data;
					8'd64: tm0_reload_q <= data;
					8'd65: tm1c_q <= data;
					8'd66: tm1_reload_q <= data;
					8'd67: begin
						tm0_reload_write_seq_q <= data[1:0];
						tm1c_write_seq_q <= data[3:2];
						tm1_reload_write_seq_q <= data[5:4];
						clkt_write_seq_q <= data[7:6];
					end
					8'd68: begin
						tm0c_write_seq_q <= data[1:0];
						clkt_run_q <= data[2];
						clkt_minute_q <= data[3];
					end
					8'd69: begin
						rtc_toggle_prev_q <= data[0];
						rtc_seen_toggle_q <= data[1];
						rtc_host_init_done_q <= data[2];
					end
					8'd70: rtc_year_q <= data;
					8'd71: rtc_month_q <= data;
					8'd72: rtc_day_q <= data;
					8'd73: rtc_hour_q <= data;
					8'd74: rtc_minute_q <= data;
					8'd75: rtc_second_q <= data;
					8'd76: uart_tx_active_q <= data[0];
					8'd77: uart_tx_div_q[7:0] <= data;
					8'd78: uart_tx_div_q[15:8] <= data;
					8'd79: uart_tx_shift_q[7:0] <= data;
					8'd80: uart_tx_shift_q[11:8] <= data[3:0];
					8'd81: uart_tx_bits_q <= data[3:0];
					8'd82: uart_rx_active_q <= data[0];
					8'd83: uart_rx_div_q[7:0] <= data;
					8'd84: uart_rx_div_q[15:8] <= data;
					8'd85: uart_rx_shift_q <= data;
					8'd86: uart_rx_state_q <= data[2:0];
					8'd87: uart_rx_bit_q <= data[3:0];
					8'd88: begin
						uart_rx_parity_q <= data[0];
						uart_rx_pe_q <= data[1];
						uart_rx_fe_q <= data[2];
						uart_rx_paren_q <= data[3];
						uart_rx_odd_q <= data[4];
						uart_rx_stop2_q <= data[5];
						rxdb_prev_q <= data[6];
						txdb_q <= data[7];
					end
					8'd89: wdt_q <= data;
					8'd90: wdtc_q <= data;
					8'd91: wdt_div_q[7:0] <= data;
					8'd92: wdt_div_q[15:8] <= data;
					8'd93: wdt_div_q[23:16] <= data;
					8'd94: opcode_q <= data;
					8'd95: operand0_q <= data;
					8'd96: operand1_q <= data;
					8'd97: mem_byte_q <= data;
					8'd98: alu_res_q <= data;
					8'd99: target_addr_q <= data;
					8'd100: begin
						pair_base_q <= data[3:0];
						target_reg_q <= data[7:4];
					end
					8'd101: op_class_q <= data[5:0];
					8'd102: op_stage_q <= data[2:0];
					8'd103: state_q <= data[5:0];
					8'd104: return_state_q <= data[5:0];
					8'd105: begin
						read_wait_q <= data[1:0];
						stall_count_q <= data[7:2];
					end
					8'd106: warmup_q[7:0] <= data;
					8'd107: warmup_q[15:8] <= data;
					8'd108: warmup_q[17:16] <= data[1:0];
					8'd109: vector_addr_q[7:0] <= data;
					8'd110: vector_addr_q[15:8] <= data;
					8'd111: access_kind_q <= data[2:0];
					8'd112: access_addr_q[7:0] <= data;
					8'd113: access_addr_q[15:8] <= data;
					8'd114: access_wdata_q <= data;
					8'd115: eff_addr_q[7:0] <= data;
					8'd116: eff_addr_q[15:8] <= data;
					8'd117: md_acc_q[7:0] <= data;
					8'd118: md_acc_q[15:8] <= data;
					8'd119: md_shift_q[7:0] <= data;
					8'd120: md_shift_q[15:8] <= data;
					8'd121: md_work_q <= data;
					8'd122: md_rem_q[7:0] <= data;
					8'd123: md_rem_q[8] <= data[0];
					8'd124: md_count_q <= data[4:0];
					8'd125: ram_addr_q[7:0] <= data;
					8'd126: ram_addr_q[9:8] <= data[1:0];
					8'd127: ram_wdata_q <= data;
					8'd128: rom_addr_q[7:0] <= data;
					8'd129: rom_addr_q[11:8] <= data[3:0];
					8'd130: begin
						clk_q <= data[0];
						warm_boot_sysflag_turnon_suppress_q <= data[1];
						nmib_prev_q <= data[2];
						intb_prev_q <= data[3];
						irq_resume_defer_q <= data[4];
						power_halt_wake_seen_q <= data[5];
					end
					8'd131: lcdc_dma_div_q[7:0] <= data;
					8'd132: lcdc_dma_div_q[15:8] <= data;
					8'd133: lcdc_dma_div_q[23:16] <= data;
					8'd134: lcdc_scan_div_q[7:0] <= data;
					8'd135: lcdc_scan_div_q[15:8] <= data;
					8'd136: lcdc_scan_div_q[23:16] <= data;
					8'd137: lcdc_shift_q <= data[5:0];
					8'd138: lcdc_hphase_q <= data[4:0];
					8'd139: lcdc_line_q <= data;
					8'd140: begin
						lcdc_vblank_q <= data[0];
						lcdc_vblank_prev_q <= data[1];
						lcdc_fr_q <= data[2];
						lcdc_lp_q <= data[3];
						lcdc_xc_q <= data[4];
						lcdc_yd_q <= data[5];
						video_vblank_prev_q <= data[6];
					end
					8'd141: lcdc_scan_byte_q <= data;
					8'd142: lcdc_xd_q <= data[3:0];
					8'd143: begin
						dma_active_q <= data[0];
						dma_hdot_200_q <= data[4];
						dma_src_phase_q <= data[7:6];
					end
					8'd148: dma_arm_src_x_q <= data;
					8'd150: dma_arm_dst_x_q <= data;
					8'd152: dma_arm_line_count_q <= data;
					8'd154: dma_ctl_q <= data;
					8'd155: dma_dmpl_q <= data;
					8'd156: dma_dmbr_q <= data;
					8'd157: dma_dmvp_q <= data;
					8'd158: dma_src_byte_q <= data;
					8'd160: dma_src_addr_q[7:0] <= data;
					8'd161: dma_src_addr_q[13:8] <= data[5:0];
					8'd162: dma_src_line_q[7:0] <= data;
					8'd163: dma_src_line_q[13:8] <= data[5:0];
					8'd164: dma_src_x_q <= data;
					8'd165: dma_src_y_q <= data;
					8'd166: dma_dst_addr_q[7:0] <= data;
					8'd167: dma_dst_addr_q[13:8] <= data[5:0];
					8'd168: dma_dst_line_q[7:0] <= data;
					8'd169: dma_dst_line_q[13:8] <= data[5:0];
					8'd170: dma_dst_x_q <= data;
					8'd171: dma_dst_y_q <= data;
					8'd172: dma_line_count_q <= data;
					8'd173: dma_row_count_q <= data;
					8'd174: dma_mode_q <= data[1:0];
					8'd175: sfr_hole_18_q <= data;
					8'd176: sfr_hole_1b_q <= data;
					8'd177: sfr_hole_29_q <= data;
					8'd178: sfr_hole_2a_q <= data;
					8'd179: sfr_hole_2f_q <= data;
					8'd180: sfr_hole_33_q <= data;
					8'd181: sfr_hole_3e_q <= data;
					8'd182: sfr_hole_3f_q <= data;
					8'd183: sfr_hole_41_q <= data;
					8'd184: sfr_hole_43_q <= data;
					8'd185: sfr_hole_45_q <= data;
					8'd186: sfr_hole_4b_q <= data;
					8'd187: sfr_hole_4f_q <= data;
					8'd188: sfr_hole_55_q <= data;
					8'd189: sfr_hole_56_q <= data;
					8'd190: sfr_hole_57_q <= data;
					8'd191: sfr_hole_58_q <= data;
					8'd192: sfr_hole_59_q <= data;
					8'd193: sfr_hole_5a_q <= data;
					8'd194: sfr_hole_5b_q <= data;
					8'd195: sfr_hole_5c_q <= data;
					8'd196: sfr_hole_5d_q <= data;
					8'd229: a_q[7:0] <= data;
					8'd230: a_q[15:8] <= data;
					8'd231: a_q[20:16] <= data[4:0];
					8'd232: d_dout_q <= data;
					8'd233: begin
						mce0b_q <= data[7];
						mce1b_q <= data[6];
						ioe0b_q <= data[5];
						ioe1b_q <= data[4];
						rdb_q <= data[3];
						wrb_q <= data[2];
						d_oe_q <= data[1];
						vd_oe_q <= data[0];
					end
					8'd234: va_q[7:0] <= data;
					8'd235: va_q[12:8] <= data[4:0];
					8'd236: vd_dout_q <= data;
					8'd237: begin
						vce0b_q <= data[0];
						vce1b_q <= data[1];
						vrdb_q <= data[2];
						vwrb_q <= data[3];
					end
					8'd238: begin
						cpu_phi1_pending_q <= data[0];
						// Codes 1 to 6 are the encoded CKC selections; 0 and 7 mean
						// the state predates the field or is corrupt, so fall back
						// to the reset clock rather than a stopped one.
						if ((data[7:5] >= 3'd1) && (data[7:5] <= 3'd6)) begin
							cpu_prescale_q <= data[4:1];
						end else begin
							cpu_prescale_q <= cpu_prescale_reload(CKC_RESET_FCPUS);
						end
						cpu_clock_select_q <= cpu_clock_select_decode(data[7:5]);
					end
					8'd239: cpu_subclock_accum_q[7:0] <= data;
					8'd240: cpu_subclock_accum_q[15:8] <= data;
					8'd241: cpu_subclock_accum_q[23:16] <= data;
					8'd242: begin
						cpu_subclock_accum_q[24] <= data[0];
						cpu_subclock_phase_q <= data[1];
					end
					8'd243: dma_src_next_byte_q <= data;
					8'd244: dma_packet_byte_q <= data;
					8'd245: begin
						dma_src_byte_valid_q <= data[0];
						dma_src_next_valid_q <= data[1];
						dma_packet_pixels_q <= data[4:2];
					end
					default: begin
					end
				endcase
			end
		end
	endtask

`ifndef SYNTHESIS
	task debug_write_sfr_hole;
		input [7:0] addr;
		input [7:0] data;
		begin
			case (addr)
				8'h18: sfr_hole_18_q = data;
				8'h1B: sfr_hole_1b_q = data;
				8'h29: sfr_hole_29_q = data;
				8'h2A: sfr_hole_2a_q = data;
				8'h2F: sfr_hole_2f_q = data;
				8'h33: sfr_hole_33_q = data;
				8'h3E: sfr_hole_3e_q = data;
				8'h3F: sfr_hole_3f_q = data;
				8'h41: sfr_hole_41_q = data;
				8'h43: sfr_hole_43_q = data;
				8'h45: sfr_hole_45_q = data;
				8'h4B: sfr_hole_4b_q = data;
				8'h4F: sfr_hole_4f_q = data;
				8'h55: sfr_hole_55_q = data;
				8'h56: sfr_hole_56_q = data;
				8'h57: sfr_hole_57_q = data;
				8'h58: sfr_hole_58_q = data;
				8'h59: sfr_hole_59_q = data;
				8'h5A: sfr_hole_5a_q = data;
				8'h5B: sfr_hole_5b_q = data;
				8'h5C: sfr_hole_5c_q = data;
				8'h5D: sfr_hole_5d_q = data;
				default: begin
				end
			endcase
		end
	endtask
`endif

`ifndef SYNTHESIS
	task refresh_gp_mirror;
		input [4:0] bank_sel_i;
		integer mirror_i;
		reg [8:0] base_v;
		begin
			base_v = {1'b0, bank_sel_i, 3'b000};
			for (mirror_i = 0; mirror_i < 16; mirror_i = mirror_i + 1) begin
				lowmem_q[mirror_i[7:0]] <= reg_bank_q[base_v + mirror_i[8:0]];
			end
		end
	endtask
`endif

	//========================================================================
	// Direct-address reads
	//========================================================================
	function [7:0] direct_read;
		input [7:0] addr;
		begin
			if (addr < 8'h10) begin
				direct_read = gp_read(addr[3:0]);
			end else if (addr[7]) begin
				direct_read = iram_lo_shadow_q[addr[6:0]];
			end else begin
				direct_read = read_sfr(addr);
			end
		end
	endfunction

	// Reading URDR clears its receive flag and reading URSR clears its error
	// flags, so a second read of 2DH collapsed into the same beat has to see
	// the cleared value rather than the port's raw one.
	function [7:0] after_sfr_reads;
		input [7:0] addr;
		input [7:0] value;
		input prior_uartr_read;
		input prior_uarts_read;
		begin
			after_sfr_reads = value;
			if (addr == 8'h2D) begin
				if (prior_uartr_read) after_sfr_reads[0] = 1'b0;
				if (prior_uarts_read) after_sfr_reads[4:2] = 3'b000;
			end
		end
	endfunction

	// Word form. Sharp requires even RR addresses; raw odd progression is only
	// a deterministic model fallback for the documented unreliable case.
	function [15:0] word_after_sfr_reads;
		input [7:0] addr;
		input [15:0] value;
		input prior_uartr_read;
		input prior_uarts_read;
		begin
			word_after_sfr_reads = {
				after_sfr_reads(addr, value[15:8], prior_uartr_read, prior_uarts_read),
				after_sfr_reads(
					addr + 8'h01,
					value[7:0],
					prior_uartr_read || (addr == 8'h2C),
					prior_uarts_read || (addr == 8'h2D)
				)
			};
		end
	endfunction

	function [15:0] gp_read_word;
		input [3:0] reg_idx;
		begin
			gp_read_word = {gp_read(reg_idx), gp_read(reg_idx + 4'd1)};
		end
	endfunction

	//========================================================================
	// Real-time clock and stack
	//========================================================================
	function rtc_iram_addr;
		input [15:0] addr;
		begin
			case (addr)
				RTC_YEAR_ADDR,
				RTC_MONTH_ADDR,
				RTC_DAY_ADDR,
				RTC_HOUR_ADDR,
				RTC_MINUTE_ADDR,
				RTC_SECOND_ADDR: rtc_iram_addr = 1'b1;
				default: rtc_iram_addr = 1'b0;
			endcase
		end
	endfunction

	function [7:0] rtc_shadow_read;
		input [15:0] addr;
		begin
			case (addr)
				RTC_YEAR_ADDR: rtc_shadow_read = rtc_year_q;
				RTC_MONTH_ADDR: rtc_shadow_read = rtc_month_q;
				RTC_DAY_ADDR: rtc_shadow_read = rtc_day_q;
				RTC_HOUR_ADDR: rtc_shadow_read = rtc_hour_q;
				RTC_MINUTE_ADDR: rtc_shadow_read = rtc_minute_q;
				RTC_SECOND_ADDR: rtc_shadow_read = rtc_second_q;
				default: rtc_shadow_read = 8'h00;
			endcase
		end
	endfunction

	function rtc_leap_year;
		input [7:0] year;
		begin
			if (year[4]) begin
				case (year[3:0])
					4'h2, 4'h6: rtc_leap_year = 1'b1;
					default: rtc_leap_year = 1'b0;
				endcase
			end else begin
				case (year[3:0])
					4'h0, 4'h4, 4'h8: rtc_leap_year = 1'b1;
					default: rtc_leap_year = 1'b0;
				endcase
			end
		end
	endfunction

	function [7:0] rtc_bcd_increment;
		input [7:0] value;
		begin
			if (value[3:0] == 4'd9) begin
				rtc_bcd_increment = {value[7:4] + 4'd1, 4'h0};
			end else begin
				rtc_bcd_increment = {value[7:4], value[3:0] + 4'd1};
			end
		end
	endfunction

	function [7:0] rtc_days_in_month;
		input [7:0] month;
		input [7:0] year;
		begin
			case (month)
				8'h01, 8'h03, 8'h05, 8'h07, 8'h08, 8'h10, 8'h12: rtc_days_in_month = 8'h31;
				8'h04, 8'h06, 8'h09, 8'h11: rtc_days_in_month = 8'h30;
				8'h02: rtc_days_in_month = rtc_leap_year(year) ? 8'h29 : 8'h28;
				default: rtc_days_in_month = 8'h31;
			endcase
		end
	endfunction

	function [15:0] stack_dec;
		input [15:0] sp_in;
		reg [15:0] dec_v;
		begin
			dec_v = sp_in - 16'h0001;
			if (!sys_q[6]) dec_v = {8'h00, dec_v[7:0]};
			stack_dec = dec_v;
		end
	endfunction

	function [15:0] stack_inc;
		input [15:0] sp_in;
		reg [15:0] inc_v;
		begin
			inc_v = sp_in + 16'h0001;
			if (!sys_q[6]) inc_v = {8'h00, inc_v[7:0]};
			stack_inc = inc_v;
		end
	endfunction

	//========================================================================
	// SFR reads
	//========================================================================
	function [7:0] read_sfr;
		input [7:0] addr;
		begin
			case (addr)
				8'h10: read_sfr = ie0_q;
				8'h11: read_sfr = ie1_q;
				8'h12: read_sfr = ir0_q;
				8'h13: read_sfr = ir1_q;
				8'h14: read_sfr = resolve_port_read(p0_q, p0c_q, p0_din_i);
				8'h15: read_sfr = resolve_port_read(p1_q, p1c_q, p1_din_i);
				8'h16: read_sfr = resolve_port_read(p2_q, p2c_q, p2_din_i);
				8'h17: read_sfr = resolve_port_read(p3_q, p3c_q, p3_din_i);
				8'h18: read_sfr = sfr_hole_18_q;
				8'h19: read_sfr = sys_q;
				8'h1A: read_sfr = ckc_q;
				8'h1B: read_sfr = sfr_hole_1b_q;
				8'h1C: read_sfr = sp_q[15:8];
				8'h1D: read_sfr = sp_q[7:0];
				8'h1E: read_sfr = ps0_q;
				8'h1F: read_sfr = ps1_q;
				8'h20: read_sfr = p0c_q;
				8'h21: read_sfr = p1c_q;
				8'h22: read_sfr = p2c_q;
				8'h23: read_sfr = p3c_q;
				8'h24: read_sfr = mmu0_q;
				8'h25: read_sfr = mmu1_q;
				8'h26: read_sfr = mmu2_q;
				8'h27: read_sfr = mmu3_q;
				8'h28: read_sfr = mmu4_q;
				8'h29: read_sfr = sfr_hole_29_q;
				8'h2A: read_sfr = sfr_hole_2a_q;
				8'h2B: read_sfr = urtt_q;
				8'h2C: read_sfr = urtr_q;
				8'h2D: read_sfr = urts_q;
				8'h2E: read_sfr = urtc_q;
				8'h2F: read_sfr = sfr_hole_2f_q;
				8'h30: read_sfr = lcc_q;
				8'h31: read_sfr = lch_q;
				8'h32: begin
					// LCV[7] is the LCDC VBlank status bit. Keep it tied to the
					// internal LCDC timing rather than the larger MiSTer host blanking
					// area around the adapted 200x160 video output.
					read_sfr = {lcdc_vblank_q, lcv_q[6:0]};
				end
				8'h33: read_sfr = sfr_hole_33_q;
				8'h34: read_sfr = dmc_q;
				8'h35: read_sfr = dmx1_q;
				8'h36: read_sfr = dmy1_q;
				8'h37: read_sfr = dmdx_q;
				8'h38: read_sfr = dmdy_q;
				8'h39: read_sfr = dmx2_q;
				8'h3A: read_sfr = dmy2_q;
				8'h3B: read_sfr = dmpl_q;
				8'h3C: read_sfr = dmbr_q;
				8'h3D: read_sfr = dmvp_q;
				8'h3E: read_sfr = sfr_hole_3e_q;
				8'h3F: read_sfr = sfr_hole_3f_q;
				8'h40: read_sfr = sgc_q;
				8'h41: read_sfr = sfr_hole_41_q;
				8'h42: read_sfr = {3'b000, sg0l_q[4:0]};
				8'h43: read_sfr = sfr_hole_43_q;
				8'h44: read_sfr = {3'b000, sg1l_q[4:0]};
				8'h45: read_sfr = sfr_hole_45_q;
				8'h46: read_sfr = {4'h0, sg0th_q[3:0]};
				8'h47: read_sfr = sg0t_q[7:0];
				8'h48: read_sfr = {4'h0, sg1th_q[3:0]};
				8'h49: read_sfr = sg1t_q[7:0];
				8'h4A: read_sfr = {3'b000, sg2l_q[4:0]};
				8'h4B: read_sfr = sfr_hole_4b_q;
				8'h4C: read_sfr = {4'h0, sg2th_q[3:0]};
				8'h4D: read_sfr = sg2t_q[7:0];
				8'h4E: read_sfr = 8'h00;
				8'h4F: read_sfr = sfr_hole_4f_q;
				8'h60, 8'h61, 8'h62, 8'h63, 8'h64, 8'h65, 8'h66, 8'h67,
				8'h68, 8'h69, 8'h6A, 8'h6B, 8'h6C, 8'h6D, 8'h6E, 8'h6F: begin
					read_sfr = sgc_q[0] ? 8'h00 : sg0w_q[addr[3:0]];
				end
				8'h70, 8'h71, 8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77,
				8'h78, 8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7E, 8'h7F: begin
					read_sfr = sgc_q[1] ? 8'h00 : sg1w_q[addr[3:0]];
				end
				8'h50: read_sfr = tm0c_q;
				8'h51: read_sfr = tm0d_q;
				8'h52: read_sfr = tm1c_q;
				8'h53: read_sfr = tm1d_q;
				8'h54: read_sfr = {clkt_run_q, clkt_minute_q, clkt_count_q};
				8'h55: read_sfr = sfr_hole_55_q;
				8'h56: read_sfr = sfr_hole_56_q;
				8'h57: read_sfr = sfr_hole_57_q;
				8'h58: read_sfr = sfr_hole_58_q;
				8'h59: read_sfr = sfr_hole_59_q;
				8'h5A: read_sfr = sfr_hole_5a_q;
				8'h5B: read_sfr = sfr_hole_5b_q;
				8'h5C: read_sfr = sfr_hole_5c_q;
				8'h5D: read_sfr = sfr_hole_5d_q;
				8'h5E: read_sfr = wdt_q;
				8'h5F: read_sfr = wdtc_q;
`ifndef SYNTHESIS
				default: read_sfr = sfr_shadow_q[addr[6:0]];
`else
				default: read_sfr = 8'h00;
`endif
			endcase
		end
	endfunction

	// Datasheet: UART baud is Timer 0 output divided by 32. Timer 0's output
	// inverts on every compare, so one output period is two compares, and one
	// compare takes the selected prescaler tap times the time constant:
	//
	//   bit period = 2 * 32 * steps * tap        (in phi0 ticks)
	//
	// The tap is a power of two, so the whole thing is a shift.
	//
	// The x2 here is the output PIN inverting on every compare, which the
	// datasheet states outright. It is unrelated to how the timer interrupt is
	// taken - that is once per compare, see sm8521_timers.v.
	//
	// GUESS: this synthesizes the average period from the registers rather than
	// counting literal tm0_out_q edges, so the phase after a TM0C write may
	// differ from hardware.
	function [15:0] uart_bit_period;
		input [7:0] timer_reload_i;
		input [2:0] timer_sel_i;
		reg [8:0] steps_v;
		reg [4:0] shift_v;
		reg [24:0] wide_v;
		begin
			// A zero time constant compares on every prescaler step.
			steps_v = (timer_reload_i == 8'h00) ? 9'd1 : {1'b0, timer_reload_i};
			// log2 of the prescaler tap in phi0 ticks: selector 000 is one
			// tick, and 001-111 are 512 through 32768.
			// Must track timer_prescale_reload in sm8521_timers.v. Selector 000
			// is one phi0 tick; 001-111 are 1024 through 65536.
			// tb_sm8521 FOCUS-UART-TAP checks the two tables still agree.
			case (timer_sel_i)
				3'b000: shift_v = 5'd0;
				3'b001: shift_v = 5'd10;
				3'b010: shift_v = 5'd11;
				3'b011: shift_v = 5'd12;
				3'b100: shift_v = 5'd13;
				3'b101: shift_v = 5'd14;
				3'b110: shift_v = 5'd15;
				default: shift_v = 5'd16;
			endcase
			// One more for the output toggle, then log2 of the fixed divider.
			case (UART_TX_BIT_PHI0_CYCLES_W)
				16'd4: shift_v = shift_v + 5'd3;
				16'd8: shift_v = shift_v + 5'd4;
				16'd16: shift_v = shift_v + 5'd5;
				16'd32: shift_v = shift_v + 5'd6;
				16'd64: shift_v = shift_v + 5'd7;
				default: shift_v = shift_v + 5'd6;
			endcase
			// A period past 16 bits is slower than about 76 baud at 5 MHz phi0,
			// which no Game.com link setting reaches. Saturate, do not wrap.
			if (shift_v >= 5'd16) begin
				uart_bit_period = 16'hFFFF;
			end else begin
				wide_v = {16'd0, steps_v} << shift_v[3:0];
				uart_bit_period = (wide_v[24:16] != 9'd0) ? 16'hFFFF : wide_v[15:0];
			end
		end
	endfunction

	wire [15:0] uart_bit_period_w = uart_bit_period(tm0_reload_q, tm0c_q[2:0]);
	wire [15:0] uart_half_bit_period_w = {1'b0, uart_bit_period_w[15:1]};

	//========================================================================
	// Address classification and CPU clock selection
	//========================================================================
	function [2:0] classify_access;
		input [15:0] addr;
		begin
			if (addr < 16'h0010) begin
				classify_access = AK_GP;
			end else if (addr < 16'h0080) begin
				classify_access = AK_SFR;
			end else if (addr < 16'h0400) begin
				classify_access = AK_IRAM;
			end else if (addr < 16'h1000) begin
				classify_access = AK_NONE;
			// The internal 4 KiB boot ROM occupies 0x1000-0x1FFF after reset
			// until the first MMU0 write, after which normal MMU0 mapping takes over.
			end else if ((addr < 16'h2000) && !mmu0_en_q) begin
				classify_access = AK_IROM;
			end else if ((addr >= 16'hA000) && (addr < 16'hE000)) begin
				classify_access = AK_VRAM;
			end else begin
				classify_access = AK_EXT;
			end
		end
	endfunction

	function [20:0] external_addr;
		input [15:0] cpu_addr;
		begin
			case (cpu_addr[15:13])
				// MMU0 selects an 8 KiB bank even though the CPU-visible window is
				// only 0x1000-0x1FFF once the boot ROM is disabled. The datasheet
				// says the CPU cannot access the lower 4 KiB of MMU0, and the
				// official emulator maps this window as (bank * 0x2000) + 0x1000.
				3'b000: external_addr = {mmu0_q, cpu_addr[12:0]};
				// The official emulator treats MMU1-4 writes as immediate window
				// base changes. BIOS service wrappers depend on MMU2 reaching bank
				// 0x26 directly rather than being filtered through cart-presence
				// policy.
				3'b001: external_addr = {mmu1_q, cpu_addr[12:0]};
				3'b010: external_addr = {mmu2_q, cpu_addr[12:0]};
				3'b011: external_addr = {mmu3_q, cpu_addr[12:0]};
				3'b100: external_addr = {mmu4_q, cpu_addr[12:0]};
				default: external_addr = {5'h00, cpu_addr};
			endcase
		end
	endfunction

	function ext_access_is_slow_rom;
		input [15:0] addr;
		reg [20:0] phys_v;
		begin
			phys_v = external_addr(addr);
			if ((addr[15:8] != 8'hFF) && (addr[15:13] <= 3'b100) && (phys_v >= 21'd262144)) begin
				ext_access_is_slow_rom = 1'b1;
			end else begin
				ext_access_is_slow_rom = 1'b0;
			end
		end
	endfunction

	function cc_true;
		input [7:0] op;
		begin
			case (op[3:0])
				4'h0: cc_true = 1'b0;
				4'h1: cc_true = (ps1_q[5] ^ ps1_q[4]);
				4'h2: cc_true = ((ps1_q[5] & ~ps1_q[4]) | (ps1_q[5] & ps1_q[4] & ps1_q[6]) | (~ps1_q[5] & (ps1_q[6] | ps1_q[4])));
				4'h3: cc_true = ps1_q[6] | ps1_q[7];
				4'h4: cc_true = ps1_q[4];
				4'h5: cc_true = ps1_q[5];
				4'h6: cc_true = ps1_q[6];
				4'h7: cc_true = ps1_q[7];
				4'h8: cc_true = 1'b1;
				4'h9: cc_true = ~(ps1_q[5] ^ ps1_q[4]);
				4'hA: cc_true = ((~ps1_q[6]) & ((ps1_q[5] & ps1_q[4]) | (~ps1_q[5] & ~ps1_q[4])));
				4'hB: cc_true = ~(ps1_q[6] | ps1_q[7]);
				4'hC: cc_true = ~ps1_q[4];
				4'hD: cc_true = ~ps1_q[5];
				4'hE: cc_true = ~ps1_q[6];
				default: cc_true = ~ps1_q[7];
			endcase
		end
	endfunction

	function [17:0] stop_warmup_cycles;
		input [1:0] warmup_select_i;
		begin
			case (warmup_select_i)
				// Datasheet WUPS values are 2^18, 2^17, 2^16, and 2^15
				// main-clock periods. The core decrements warmup_q on phi1,
				// so these are half-counts in the fCK/2 tick domain.
				// CKC[7]/CKC[5:3] select the CPU/system clock latched by STOP.
				// That divider does not scale the timer, watchdog or audio domains.
				2'b00: stop_warmup_cycles = 18'd131072;
				2'b01: stop_warmup_cycles = 18'd65536;
				2'b10: stop_warmup_cycles = 18'd32768;
				default: stop_warmup_cycles = 18'd16384;
			endcase
		end
	endfunction

	// Savestate coding for the CPU clock selection. Code 0 means "no clock
	// record", so a state saved before this field existed restores the reset
	// clock instead of silently dropping to fCK/32. The two reserved CKC
	// selections have no code at all, so a corrupt byte cannot stop the clock.
	function [2:0] cpu_clock_select_encode;
		input [2:0] select_i;
		begin
			case (select_i)
				3'b000: cpu_clock_select_encode = 3'd1;
				3'b001: cpu_clock_select_encode = 3'd2;
				3'b010: cpu_clock_select_encode = 3'd3;
				3'b011: cpu_clock_select_encode = 3'd4;
				3'b100: cpu_clock_select_encode = 3'd5;
				3'b111: cpu_clock_select_encode = 3'd6;
				default: cpu_clock_select_encode = 3'd0;
			endcase
		end
	endfunction

	function [2:0] cpu_clock_select_decode;
		input [2:0] code_i;
		begin
			case (code_i)
				3'd1: cpu_clock_select_decode = 3'b000;
				3'd2: cpu_clock_select_decode = 3'b001;
				3'd3: cpu_clock_select_decode = 3'b010;
				3'd4: cpu_clock_select_decode = 3'b011;
				3'd5: cpu_clock_select_decode = 3'b100;
				3'd6: cpu_clock_select_decode = 3'b111;
				default: cpu_clock_select_decode = CKC_RESET_FCPUS;
			endcase
		end
	endfunction

	function [3:0] cpu_prescale_reload;
		input [2:0] clock_select_i;
		begin
			case (clock_select_i)
				3'b000: cpu_prescale_reload = 4'd15; // fCK/32
				3'b001: cpu_prescale_reload = 4'd7;  // fCK/16
				3'b010: cpu_prescale_reload = 4'd3;  // fCK/8
				3'b011: cpu_prescale_reload = 4'd1;  // fCK/4
				default: cpu_prescale_reload = 4'd0; // fCK/2
			endcase
		end
	endfunction

	// MAME keeps a private pending bitmap (`m_IFLAGS`) alongside the
	// software-visible IR0/IR1 request bits. The private request determines
	// delivery; IR0/IR1 remain status latches.
	wire [3:0] pending_irq_source_w =
		irq_dma_pending_q ? IRQ_DMA :
		irq_tim0_pending_q ? IRQ_TIM0 :
		ext_pending_q ? IRQ_EXT :
		irq_uart_pending_q ? IRQ_UART :
		irq_lcdc_pending_q ? IRQ_LCDC :
		irq_tim1_pending_q ? IRQ_TIM1 :
		irq_clk_pending_q ? IRQ_CLK :
		irq_pio_pending_q ? IRQ_PIO :
		IRQ_NONE;

	// IE, global-I, and the current priority gate acceptance without altering
	// the private pending request above.
	wire [3:0] takeable_irq_source_w =
		(irq_dma_pending_q && ie0_q[7] && ps1_q[0]) ? IRQ_DMA :
		(irq_tim0_pending_q && ie0_q[6] && ps1_q[0]) ? IRQ_TIM0 :
		(ext_pending_q && ie0_q[4] && ps1_q[0] && (ps0_q[2:0] < 3'd7)) ? IRQ_EXT :
		(irq_uart_pending_q && ie0_q[3] && ps1_q[0] && (ps0_q[2:0] < 3'd6)) ? IRQ_UART :
		(irq_lcdc_pending_q && ie0_q[0] && ps1_q[0] && (ps0_q[2:0] < 3'd5)) ? IRQ_LCDC :
		(irq_tim1_pending_q && ie1_q[6] && ps1_q[0] && (ps0_q[2:0] < 3'd4)) ? IRQ_TIM1 :
		(irq_clk_pending_q && ie1_q[4] && ps1_q[0] && (ps0_q[2:0] < 3'd3)) ? IRQ_CLK :
		(irq_pio_pending_q && ie1_q[2] && ps1_q[0] && (ps0_q[2:0] < 3'd2)) ? IRQ_PIO :
		IRQ_NONE;

	// HALT is the shallow standby mode. The datasheet says every interrupt
	// source except the illegal-instruction trap can wake HALT, unlike STOP
	// which accepts only a few; the IE0/IE1 text adds that the source must be
	// enabled. Both hold together as "any source, once enabled", so IE gates
	// the wake here while global-I and priority gate vector acceptance.
	wire [3:0] halt_wake_irq_source_w =
		(irq_dma_pending_q && ie0_q[7]) ? IRQ_DMA :
		(irq_tim0_pending_q && ie0_q[6]) ? IRQ_TIM0 :
		(ext_pending_q && ie0_q[4]) ? IRQ_EXT :
		(irq_uart_pending_q && ie0_q[3]) ? IRQ_UART :
		(irq_lcdc_pending_q && ie0_q[0]) ? IRQ_LCDC :
		(irq_tim1_pending_q && ie1_q[6]) ? IRQ_TIM1 :
		(irq_clk_pending_q && ie1_q[4]) ? IRQ_CLK :
		(irq_pio_pending_q && ie1_q[2]) ? IRQ_PIO :
		IRQ_NONE;

	// GUESS: STOP wake is limited to sources left operable by the datasheet.
	// UART/SIO can wake from an already-pending request; exact serial activity
	// while its block is stopped is not documented well enough to model.
	wire [3:0] stop_wake_irq_source_w =
		(ext_pending_q && ie0_q[4]) ? IRQ_EXT :
		(irq_uart_pending_q && ie0_q[3]) ? IRQ_UART :
		(irq_clk_pending_q && ie1_q[4]) ? IRQ_CLK :
		IRQ_NONE;

	//========================================================================
	// Instruction timing
	//========================================================================
	// Extra read beats beyond the documented 2-cycle access. The setup beat
	// plus the sample beat already covers IRAM, VRAM and ordinary external
	// reads: the setup beat drives the registered VRAM address and strobes
	// early enough for the block RAM to settle before the next phi0, and SDRAM
	// cart ROM latency is modelled by holding the sample beat until
	// rom_read_ready_i instead. No access kind needs an extra middle beat
	// today, so read_wait_q stays at zero and the wait states it drives are
	// the hook for a kind that one day does.
	function [1:0] read_wait_states_kind;
		input [2:0] kind;
		begin
			read_wait_states_kind = 2'd0;
		end
	endfunction

	function sample_wait_for_rom;
		input [2:0] access_kind_i;
		input [15:0] access_addr_i;
		input rom_ready_i;
		begin
			sample_wait_for_rom =
				(access_kind_i == AK_EXT) &&
				ext_access_is_slow_rom(access_addr_i) &&
				!rom_ready_i;
		end
	endfunction

	// Pocket cartridge storage is asynchronous to the DMA engine. A physical
	// DMA read bypasses the MMU and leaves access_addr_q at zero, so the CPU's
	// logical-address wait helper cannot classify it. Hold only active external
	// cartridge reads; BIOS, SRAM and VRAM retain their original timing.
	wire dma_wait_for_rom_w = (access_kind_q == AK_EXT) && !mce0b_q && !rdb_q &&
		(a_q >= 21'h040000) && !rom_read_ready_i;

	function [5:0] core_fetch_resume_stall;
		input [3:0] extra_phi0_cycles;
		begin
			if (extra_phi0_cycles != 4'd0) begin
				core_fetch_resume_stall = {2'b00, extra_phi0_cycles} - 6'd1;
			end else begin
				core_fetch_resume_stall = 6'd0;
			end
		end
	endfunction

	function [3:0] rmb_fetch_resume_cycles;
		input [7:0] opcode;
		input [7:0] desc;
		begin
			if (opcode <= 8'h27) begin
				case (desc[7:6])
					2'b00: rmb_fetch_resume_cycles = 4'd3;
					2'b01: rmb_fetch_resume_cycles = 4'd4;
					2'b10: rmb_fetch_resume_cycles = (desc[2:0] == 3'b000) ? 4'd4 : 4'd2;
					default: rmb_fetch_resume_cycles = 4'd5;
				endcase
			end else if (opcode == 8'h28) begin
				case (desc[7:6])
					2'b00: rmb_fetch_resume_cycles = 4'd2;
					2'b01: rmb_fetch_resume_cycles = 4'd3;
					2'b10: rmb_fetch_resume_cycles = (desc[2:0] == 3'b000) ? 4'd4 : 4'd1;
					default: rmb_fetch_resume_cycles = 4'd4;
				endcase
			end else begin
				case (desc[7:6])
					2'b00, 2'b01: rmb_fetch_resume_cycles = 4'd4;
					2'b10: rmb_fetch_resume_cycles = (desc[2:0] == 3'b000) ? 4'd4 : 4'd3;
					default: rmb_fetch_resume_cycles = 4'd5;
				endcase
			end
		end
	endfunction

	function [3:0] rmw_fetch_resume_cycles;
		input [7:0] opcode;
		input [7:0] desc;
		begin
			case (desc[7:6])
				2'b00: rmw_fetch_resume_cycles = 4'd2;
				// MOV R,(RR)+ overlaps the pair increment with read completion.
				2'b01: rmw_fetch_resume_cycles =
					(opcode == 8'h38) ? 4'd2 : 4'd7;
				2'b11: rmw_fetch_resume_cycles = 4'd7;
				// Keep indexed word-address forms above absolute forms, matching
				// the surrounding descriptor ladders and contemporary MCU ratios.
				default: rmw_fetch_resume_cycles =
					(desc[2:0] == 3'b000) ? 4'd1 : 4'd5;
			endcase
		end
	endfunction

	function [3:0] smw_fetch_resume_cycles;
		input [7:0] desc;
		begin
			case (desc[7:6])
				2'b00: smw_fetch_resume_cycles = 4'd3;
				2'b01, 2'b11: smw_fetch_resume_cycles = 4'd8;
				2'b10: smw_fetch_resume_cycles = (desc[2:0] == 3'b000) ? 4'd2 : 4'd6;
				default: smw_fetch_resume_cycles = 4'd0;
			endcase
		end
	endfunction

	// Post-increment and pre-decrement write the pointer register back. Only
	// the first stage of an indirect class does this; the indexed form leaves
	// its pointer alone.
	task update_rmb_pointer;
		begin
			case (operand0_q[7:6])
				2'b01: gp_write({1'b0, operand0_q[2:0]}, rmb_ptr_v + 8'h01);
				2'b11: gp_write({1'b0, operand0_q[2:0]}, rmb_addr_v);
				default: begin
				end
			endcase
		end
	endtask

	// Two ways to push. The execute stage hands the byte to the memory write
	// states through eff_addr_q and mem_byte_q; interrupt entry owns the bus
	// for its beats and drives the write itself.
	task push_byte;
		input [7:0] data;
		reg [15:0] next_sp_v;
		begin
			next_sp_v = stack_dec(sp_q);
			sp_q <= next_sp_v;
			eff_addr_q <= next_sp_v;
			mem_byte_q <= data;
		end
	endtask

	task push_byte_now;
		input [7:0] data;
		reg [15:0] next_sp_v;
		begin
			idle_bus();
			next_sp_v = stack_dec(sp_q);
			sp_q <= next_sp_v;
			begin_write(next_sp_v, data);
		end
	endtask

	// Reset the UART receiver. A reset or a control write clears the frame
	// format too; the receiver's own restart reloads it from URTC.
	task reset_uart_rx;
		input reload_format;
		begin
			uart_rx_shift_q <= 8'h00;
			uart_rx_state_q <= 3'd0;
			uart_rx_bit_q <= 4'd0;
			uart_rx_parity_q <= 1'b0;
			uart_rx_pe_q <= 1'b0;
			uart_rx_fe_q <= 1'b0;
			uart_rx_paren_q <= reload_format ? ~urtc_q[2] : 1'b0;
			uart_rx_odd_q <= reload_format ? urtc_q[1] : 1'b0;
			uart_rx_stop2_q <= reload_format ? urtc_q[0] : 1'b0;
		end
	endtask

	task enter_core_stall;
		input [5:0] stall_count;
		input [5:0] next_state;
		begin
			stall_count_q <= stall_count;
			return_state_q <= next_state;
			state_q <= ST_STALL;
		end
	endtask

	function [15:0] irq_vector_addr;
		input [3:0] irq_src;
		begin
			case (irq_src)
				IRQ_DMA:  irq_vector_addr = 16'h1000;
				IRQ_TIM0: irq_vector_addr = 16'h1002;
				IRQ_EXT:  irq_vector_addr = 16'h1006;
				IRQ_UART: irq_vector_addr = 16'h1008;
				IRQ_LCDC: irq_vector_addr = 16'h100E;
				IRQ_TIM1: irq_vector_addr = 16'h1012;
				IRQ_CLK:  irq_vector_addr = 16'h1016;
				IRQ_PIO:  irq_vector_addr = 16'h101A;
				default:  irq_vector_addr = 16'h0000;
			endcase
		end
	endfunction

	function [23:0] wdt_tick_period;
		input [2:0] sel;
		begin
			case (sel)
				3'b000: wdt_tick_period = WDT_FC12_PHI0_CYCLES_W;
				3'b001: wdt_tick_period = (WDT_FC12_PHI0_CYCLES_W << 1);
				3'b010: wdt_tick_period = (WDT_FC12_PHI0_CYCLES_W << 2);
				3'b011: wdt_tick_period = (WDT_FC12_PHI0_CYCLES_W << 3);
				3'b100: wdt_tick_period = WDT_FX5_PHI0_CYCLES_W;
				3'b101: wdt_tick_period = (WDT_FX5_PHI0_CYCLES_W << 1);
				3'b110: wdt_tick_period = (WDT_FX5_PHI0_CYCLES_W << 2);
				default: wdt_tick_period = (WDT_FX5_PHI0_CYCLES_W << 3);
			endcase
		end
	endfunction

	function [23:0] lcdc_dma_tick_period;
		input [2:0] sel;
		begin
			case (sel)
				3'b000: lcdc_dma_tick_period = 24'd1;	// fCK/2
				3'b001: lcdc_dma_tick_period = 24'd2;	// fCK/4
				3'b010: lcdc_dma_tick_period = 24'd3;	// fCK/6
				3'b011: lcdc_dma_tick_period = 24'd4;	// fCK/8
				3'b100: lcdc_dma_tick_period = 24'd5;	// fCK/10
				3'b101: lcdc_dma_tick_period = 24'd6;	// fCK/12
				3'b110: lcdc_dma_tick_period = 24'd7;	// fCK/14
				default: lcdc_dma_tick_period = 24'd8;	// fCK/16
			endcase
		end
	endfunction

	//========================================================================
	// LCDC scanout and DMA geometry
	//========================================================================
	function [6:0] vram_line_bytes_sel;
		input hdot_200_i;
		begin
			if (hdot_200_i) begin
				vram_line_bytes_sel = 7'd50;
			end else begin
				vram_line_bytes_sel = 7'd40;
			end
		end
	endfunction

	function [5:0] lcdc_shift_clocks;
		input hdot_size_i;
		begin
			if (hdot_size_i) begin
				lcdc_shift_clocks = 6'd50;
			end else begin
				lcdc_shift_clocks = 6'd40;
			end
		end
	endfunction

	function [7:0] lcdc_active_lines;
		input [1:0] vline_size_i;
		begin
			case (vline_size_i)
				2'b00: lcdc_active_lines = 8'd100;
				2'b01: lcdc_active_lines = 8'd160;
				default: lcdc_active_lines = 8'd200;
			endcase
		end
	endfunction

`ifndef SYNTHESIS
	// Diagnostic bounds helpers retained for the DMA conformance bench. The
	// hardware launch path accepts the raw coordinate registers, as documented.
	function [8:0] dma_limit_x;
		input [1:0] mode_i;
		input hdot_200_i;
		begin
			if (mode_i == 2'b11) begin
				dma_limit_x = 9'd256;
			end else if (hdot_200_i) begin
				dma_limit_x = 9'd200;
			end else begin
				dma_limit_x = 9'd160;
			end
		end
	endfunction

	function [8:0] dma_limit_y;
		input [1:0] mode_i;
		input [7:0] active_lines_i;
		begin
			if (mode_i == 2'b11) begin
				dma_limit_y = 9'd256;
			end else begin
				dma_limit_y = {1'b0, active_lines_i};
			end
		end
	endfunction
`endif

	function [13:0] dma_source_width_bytes;
		input [1:0] mode_i;
		begin
			case (mode_i)
				2'b01,
				2'b10: dma_source_width_bytes = 14'd64;
				default: dma_source_width_bytes = {7'h00, vram_line_bytes_sel(dma_hdot_200_q)};
			endcase
		end
	endfunction

	function [2:0] dma_packet_pixel_count;
		input [1:0] dst_phase_i;
		input [7:0] line_count_i;
		reg [2:0] boundary_v;
		begin
			case (dst_phase_i)
				2'd0: boundary_v = 3'd4;
				2'd1: boundary_v = 3'd3;
				2'd2: boundary_v = 3'd2;
				default: boundary_v = 3'd1;
			endcase
			if (line_count_i < {5'b00000, boundary_v}) begin
				dma_packet_pixel_count = {1'b0, line_count_i[1:0]} + 3'd1;
			end else begin
				dma_packet_pixel_count = boundary_v;
			end
		end
	endfunction

	function [2:0] dma_source_pixels_available;
		input [1:0] src_phase_i;
		input src_dec_i;
		begin
			if (src_dec_i) begin
				dma_source_pixels_available = {1'b0, src_phase_i} + 3'd1;
			end else begin
				case (src_phase_i)
					2'd0: dma_source_pixels_available = 3'd4;
					2'd1: dma_source_pixels_available = 3'd3;
					2'd2: dma_source_pixels_available = 3'd2;
					default: dma_source_pixels_available = 3'd1;
				endcase
			end
		end
	endfunction

	function dma_packet_needs_next_source;
		input [1:0] src_phase_i;
		input [2:0] packet_pixels_i;
		input src_dec_i;
		begin
			dma_packet_needs_next_source =
				packet_pixels_i > dma_source_pixels_available(src_phase_i, src_dec_i);
		end
	endfunction

	function dma_packet_advances_source_byte;
		input [1:0] src_phase_i;
		input [2:0] packet_pixels_i;
		input src_dec_i;
		begin
			dma_packet_advances_source_byte =
				packet_pixels_i >= dma_source_pixels_available(src_phase_i, src_dec_i);
		end
	endfunction

	function [1:0] dma_advance_source_phase_packet;
		input [1:0] src_phase_i;
		input [2:0] packet_pixels_i;
		input src_dec_i;
		begin
			if (src_dec_i) begin
				dma_advance_source_phase_packet = src_phase_i - packet_pixels_i[1:0];
			end else begin
				dma_advance_source_phase_packet = src_phase_i + packet_pixels_i[1:0];
			end
		end
	endfunction

	function [7:0] dma_advance_source_x_packet;
		input [7:0] src_x_i;
		input [2:0] packet_pixels_i;
		input src_dec_i;
		begin
			if (src_dec_i) begin
				dma_advance_source_x_packet = src_x_i - {5'b00000, packet_pixels_i};
			end else begin
				dma_advance_source_x_packet = src_x_i + {5'b00000, packet_pixels_i};
			end
		end
	endfunction

	function [7:0] dma_source_packet;
		input [7:0] current_byte_i;
		input [7:0] next_byte_i;
		input [1:0] src_phase_i;
		input src_dec_i;
		begin
			if (src_dec_i) begin
				case (src_phase_i)
					2'd0: dma_source_packet = {current_byte_i[7:6], next_byte_i[1:0], next_byte_i[3:2], next_byte_i[5:4]};
					2'd1: dma_source_packet = {current_byte_i[5:4], current_byte_i[7:6], next_byte_i[1:0], next_byte_i[3:2]};
					2'd2: dma_source_packet = {current_byte_i[3:2], current_byte_i[5:4], current_byte_i[7:6], next_byte_i[1:0]};
					default: dma_source_packet = {current_byte_i[1:0], current_byte_i[3:2], current_byte_i[5:4], current_byte_i[7:6]};
				endcase
			end else begin
				case (src_phase_i)
					2'd0: dma_source_packet = current_byte_i;
					2'd1: dma_source_packet = {current_byte_i[5:0], next_byte_i[7:6]};
					2'd2: dma_source_packet = {current_byte_i[3:0], next_byte_i[7:4]};
					default: dma_source_packet = {current_byte_i[1:0], next_byte_i[7:2]};
				endcase
			end
		end
	endfunction

	function [7:0] dma_next_source_y;
		input [7:0] y_i;
		input dec_i;
		begin
			// DMY1 is an 8-bit source coordinate register. Keep the raw 8-bit
			// value and let the packed-byte source address alias naturally.
			if (dec_i) begin
				dma_next_source_y = y_i - 8'h01;
			end else begin
				dma_next_source_y = y_i + 8'h01;
			end
		end
	endfunction

	function [13:0] dma_source_next_byte_addr;
		input [13:0] addr_i;
		input dec_i;
		reg [14:0] tmp_v;
		begin
			if (dec_i) begin
				tmp_v = {1'b0, addr_i} - 15'd1;
			end else begin
				tmp_v = {1'b0, addr_i} + 15'd1;
			end
			dma_source_next_byte_addr = tmp_v[13:0];
		end
	endfunction

	function [13:0] dma_source_next_line_addr;
		input [13:0] addr_i;
		input [1:0] mode_i;
		input dec_i;
		reg [14:0] tmp_v;
		reg [13:0] width_v;
		begin
			width_v = dma_source_width_bytes(mode_i);
			if (dec_i) begin
				tmp_v = {1'b0, addr_i} - {1'b0, width_v};
			end else begin
				tmp_v = {1'b0, addr_i} + {1'b0, width_v};
			end
			dma_source_next_line_addr = tmp_v[13:0];
		end
	endfunction

	function [13:0] dma_dest_width_bytes;
		input [1:0] mode_i;
		begin
			if (mode_i == 2'b11) begin
				dma_dest_width_bytes = 14'd64;
			end else begin
				dma_dest_width_bytes = {7'h00, vram_line_bytes_sel(dma_hdot_200_q)};
			end
		end
	endfunction

	function [13:0] dma_dest_next_byte_addr;
		input [13:0] addr_i;
		reg [14:0] tmp_v;
		begin
			tmp_v = {1'b0, addr_i} + 15'd1;
			dma_dest_next_byte_addr = tmp_v[13:0];
		end
	endfunction

	function [13:0] dma_dest_next_line_addr;
		input [13:0] addr_i;
		input [1:0] mode_i;
		reg [14:0] tmp_v;
		begin
			tmp_v = {1'b0, addr_i} + {1'b0, dma_dest_width_bytes(mode_i)};
			dma_dest_next_line_addr = tmp_v[13:0];
		end
	endfunction

`ifndef SYNTHESIS
	function dma_start_invalid;
		input [1:0] mode_i;
		input hdot_200_i;
		input [7:0] active_lines_i;
		reg [8:0] limit_x_v;
		reg [8:0] limit_y_v;
		reg src_invalid_v;
		reg dst_invalid_v;
		begin
			// Diagnostic helper only. The datasheet gives 8-bit coordinate and
			// size registers, but does not say out-of-range starts are refused;
			// the launch path therefore does not call this helper.
			limit_x_v = dma_limit_x(mode_i, hdot_200_i);
			limit_y_v = dma_limit_y(mode_i, active_lines_i);
			src_invalid_v = ({1'b0, dmx1_q} >= limit_x_v) || ({1'b0, dmy1_q} >= limit_y_v);
			dst_invalid_v = ({1'b0, dmx2_q} >= limit_x_v) || ({1'b0, dmy2_q} >= limit_y_v);
			case (mode_i)
				2'b00: dma_start_invalid = src_invalid_v || dst_invalid_v;
				2'b01,
				2'b10: dma_start_invalid = dst_invalid_v;
				default: dma_start_invalid = src_invalid_v;
			endcase
		end
	endfunction
`endif

	function [12:0] vram_dma_addr;
		input [7:0] x_pos;
		input [7:0] y_pos;
		reg [12:0] line_base_v;
		begin
			if (lch_q[5]) begin
				line_base_v = ({5'h00, y_pos} << 5) + ({5'h00, y_pos} << 4) + ({5'h00, y_pos} << 1);
			end else begin
				line_base_v = ({5'h00, y_pos} << 5) + ({5'h00, y_pos} << 3);
			end
			vram_dma_addr = line_base_v + {5'h00, x_pos};
		end
	endfunction

	function [3:0] lcdc_scan_xd;
		input [7:0] src_byte;
		input phase_i;
		begin
			if (phase_i) begin
				lcdc_scan_xd = {src_byte[6], src_byte[4], src_byte[2], src_byte[0]};
			end else begin
				lcdc_scan_xd = {src_byte[7], src_byte[5], src_byte[3], src_byte[1]};
			end
		end
	endfunction

	function [15:0] dma_vram_byte_cpu_addr;
		input page_sel;
		input [12:0] byte_addr;
		begin
			if (page_sel) begin
				dma_vram_byte_cpu_addr = 16'hC000 + {3'b000, byte_addr};
			end else begin
				dma_vram_byte_cpu_addr = 16'hA000 + {3'b000, byte_addr};
			end
		end
	endfunction

	function [15:0] dma_ext_ram_byte_cpu_addr;
		input [12:0] byte_addr;
		begin
			dma_ext_ram_byte_cpu_addr = 16'hE000 + {3'b000, byte_addr};
		end
	endfunction

	function [13:0] dma_dest_byte_addr;
		input [1:0] mode_i;
		input [7:0] x_pos;
		input [7:0] y_pos;
		input hdot_200_i;
		reg [13:0] addr_v;
		begin
			if (mode_i == 2'b11) begin
				addr_v = ({6'h00, y_pos} << 6) + {8'h00, x_pos[7:2]};
			end else if (hdot_200_i) begin
				addr_v = ({6'h00, y_pos} << 5) + ({6'h00, y_pos} << 4) + ({6'h00, y_pos} << 1) + {8'h00, x_pos[7:2]};
			end else begin
				addr_v = ({6'h00, y_pos} << 5) + ({6'h00, y_pos} << 3) + {8'h00, x_pos[7:2]};
			end
			dma_dest_byte_addr = addr_v;
		end
	endfunction

	function [15:0] dma_dest_cpu_addr;
		input [1:0] mode_i;
		input page_sel;
		input [13:0] byte_addr;
		begin
			if (mode_i == 2'b11) begin
				dma_dest_cpu_addr = dma_ext_ram_byte_cpu_addr(byte_addr[12:0]);
			end else begin
				dma_dest_cpu_addr = dma_vram_byte_cpu_addr(page_sel, byte_addr[12:0]);
			end
		end
	endfunction

	function [13:0] dma_source_byte_addr;
		input [1:0] mode_i;
		input [7:0] x_pos;
		input [7:0] y_pos;
		input hdot_200_i;
		reg [13:0] addr_v;
		begin
			case (mode_i)
				2'b01,
				2'b10: addr_v = ({6'h00, y_pos} << 6) + {8'h00, x_pos[7:2]};
				default: begin
					if (hdot_200_i) begin
						addr_v = ({6'h00, y_pos} << 5) + ({6'h00, y_pos} << 4) + ({6'h00, y_pos} << 1) + {8'h00, x_pos[7:2]};
					end else begin
						addr_v = ({6'h00, y_pos} << 5) + ({6'h00, y_pos} << 3) + {8'h00, x_pos[7:2]};
					end
				end
			endcase
			dma_source_byte_addr = addr_v;
		end
	endfunction

	function [7:0] packed_pixel_set;
		input [7:0] dst_byte;
		input [1:0] pixel_idx;
		input [1:0] pixel_val;
		begin
			// pixel_idx 0..3 writes byte lanes [7:6], [5:4], [3:2], [1:0],
			// matching MAME's destination lane order as well.
			case (pixel_idx)
				2'd0: packed_pixel_set = {pixel_val, dst_byte[5:0]};
				2'd1: packed_pixel_set = {dst_byte[7:6], pixel_val, dst_byte[3:0]};
				2'd2: packed_pixel_set = {dst_byte[7:4], pixel_val, dst_byte[1:0]};
				default: packed_pixel_set = {dst_byte[7:2], pixel_val};
			endcase
		end
	endfunction

`ifndef SYNTHESIS
	// Retained for the DMA conformance benches; active DMA packet assembly no
	// longer needs a separate full-byte reversal stage.
	function [7:0] reverse_dot_pairs;
		input [7:0] data;
		begin
			reverse_dot_pairs = {data[1:0], data[3:2], data[5:4], data[7:6]};
		end
	endfunction
`endif

	function [1:0] dma_map_color;
		input [7:0] dmpl_value;
		input [1:0] src_color;
		begin
			// DMPL register layout per the datasheet register-format label and example:
			//   COL31 COL30  COL21 COL20  COL11 COL10  COL01 COL00
			//     7     6      5     4      3     2      1     0
			// COL3 (bits[7:6]) = output for source color 3
			// COL0 (bits[1:0]) = output for source color 0
			//
			// The datasheet example confirms: "dot data color 2 selects bits 4 and 5
			// (= COL2 field)." This matches MAME's bit-ordering, and the BIOS programs
			// dmpl=0xe4 (11100100) as an identity map (0->0,1->1,2->2,3->3).
			// Real hardware `dma_lcdc_visual_test` captures also show DMPL=E4 as
			// identity and DMPL=1B as reversed shade mapping, so the contradictory
			// text paragraph is treated as an OCR/prose-order error.
			case (src_color)
				2'b00: dma_map_color = dmpl_value[1:0];
				2'b01: dma_map_color = dmpl_value[3:2];
				2'b10: dma_map_color = dmpl_value[5:4];
				default: dma_map_color = dmpl_value[7:6];
			endcase
		end
	endfunction

	function [7:0] dma_map_byte;
		input [7:0] dmpl_value;
		input [7:0] src_byte;
		reg [1:0] src0_v;
		reg [1:0] src1_v;
		reg [1:0] src2_v;
		reg [1:0] src3_v;
		begin
			src0_v = src_byte[7:6];
			src1_v = src_byte[5:4];
			src2_v = src_byte[3:2];
			src3_v = src_byte[1:0];
			dma_map_byte = {
				dma_map_color(dmpl_value, src0_v),
				dma_map_color(dmpl_value, src1_v),
				dma_map_color(dmpl_value, src2_v),
				dma_map_color(dmpl_value, src3_v)
			};
		end
	endfunction

	function [7:0] dma_compound_merge;
		input [7:0] dmpl_value;
		input [7:0] src_byte;
		input [7:0] dst_byte;
		reg [1:0] src0_v;
		reg [1:0] src1_v;
		reg [1:0] src2_v;
		reg [1:0] src3_v;
		reg [1:0] dst0_v;
		reg [1:0] dst1_v;
		reg [1:0] dst2_v;
		reg [1:0] dst3_v;
		begin
			src0_v = src_byte[7:6];
			src1_v = src_byte[5:4];
			src2_v = src_byte[3:2];
			src3_v = src_byte[1:0];
			dst0_v = (src0_v == 2'b00) ? dst_byte[7:6] : dma_map_color(dmpl_value, src0_v);
			dst1_v = (src1_v == 2'b00) ? dst_byte[5:4] : dma_map_color(dmpl_value, src1_v);
			dst2_v = (src2_v == 2'b00) ? dst_byte[3:2] : dma_map_color(dmpl_value, src2_v);
			dst3_v = (src3_v == 2'b00) ? dst_byte[1:0] : dma_map_color(dmpl_value, src3_v);
			dma_compound_merge = {dst0_v, dst1_v, dst2_v, dst3_v};
		end
	endfunction

	function [7:0] dma_packet_merge;
		input [7:0] dmpl_value;
		input [7:0] src_packet_i;
		input [7:0] dst_byte_i;
		input [1:0] dst_phase_i;
		input [2:0] packet_pixels_i;
		input overwrite_i;
		reg [7:0] out_v;
		begin
			out_v = dst_byte_i;
			if ((packet_pixels_i >= 3'd1) && (overwrite_i || (src_packet_i[7:6] != 2'b00))) begin
				out_v = packed_pixel_set(out_v, dst_phase_i, dma_map_color(dmpl_value, src_packet_i[7:6]));
			end
			if ((packet_pixels_i >= 3'd2) && (overwrite_i || (src_packet_i[5:4] != 2'b00))) begin
				out_v = packed_pixel_set(out_v, dst_phase_i + 2'd1, dma_map_color(dmpl_value, src_packet_i[5:4]));
			end
			if ((packet_pixels_i >= 3'd3) && (overwrite_i || (src_packet_i[3:2] != 2'b00))) begin
				out_v = packed_pixel_set(out_v, dst_phase_i + 2'd2, dma_map_color(dmpl_value, src_packet_i[3:2]));
			end
			if ((packet_pixels_i >= 3'd4) && (overwrite_i || (src_packet_i[1:0] != 2'b00))) begin
				out_v = packed_pixel_set(out_v, dst_phase_i + 2'd3, dma_map_color(dmpl_value, src_packet_i[1:0]));
			end
			dma_packet_merge = out_v;
		end
	endfunction

	function uart_parity_bit;
		input [7:0] data;
		begin
			if (!urtc_q[1]) begin
				uart_parity_bit = ^data;
			end else begin
				uart_parity_bit = ~(^data);
			end
		end
	endfunction

	reg [7:0] access_rdata_raw_w;
	always @* begin
		case (access_kind_q)
			AK_GP: access_rdata_raw_w = gp_read(access_addr_q[3:0]);
			AK_SFR: access_rdata_raw_w = read_sfr(access_addr_q[7:0]);
			AK_IRAM: begin
				if (rtc_iram_addr(access_addr_q)) begin
					access_rdata_raw_w = rtc_shadow_read(access_addr_q);
				end else if (access_addr_q[15:8] == 8'h00) begin
					access_rdata_raw_w = iram_lo_shadow_q[access_addr_q[6:0]];
				end else begin
					access_rdata_raw_w = ram_rdata_w;
				end
			end
			AK_IROM: access_rdata_raw_w = rom_rdata_w;
			AK_EXT: access_rdata_raw_w = d_din_i;
			AK_VRAM: begin
				// Game.com exposes VRAM as CPU-write-only; MAME returns 00H
				// for CPU reads, while the internal DMA engine can still read
				// VRAM for VRAM-source and destination-RMW transfers.
				case (state_q)
					ST_DMA_READ_SAMPLE,
					ST_DMA_WINDOW_SAMPLE,
					ST_DMA_DEST_SAMPLE: access_rdata_raw_w = vd_din_i;
					default: access_rdata_raw_w = 8'h00;
				endcase
			end
			default: access_rdata_raw_w = 8'h00;
		endcase
	end

	wire [7:0] cheat_gp_addr_w =
		(ps0_q & 8'hF8) + {4'h0, access_addr_q[3:0]};
	wire [15:0] cheat_addr_w = (access_kind_q == AK_GP) ?
		{8'h00, cheat_gp_addr_w} : access_addr_q;
	wire [7:0] access_rdata_cheat_w;

	gamecom_cheat_engine #(
		.MAX_CODES(12)
	) u_cheat_engine (
		.clk_sys_i(clk_sys_i),
		.clear_i(cheat_clear_i),
		.enable_i(1'b1),
		.code_i(cheat_code_i),
		.addr_i(cheat_addr_w),
		.data_i(access_rdata_raw_w),
		.data_o(access_rdata_cheat_w),
		.available_o()
	);

	//========================================================================
	// Bus access
	//========================================================================
	task idle_bus;
		begin
			a_q <= 21'h000000;
			d_dout_q <= 8'h00;
			d_oe_q <= 1'b0;
			mce0b_q <= 1'b1;
			mce1b_q <= 1'b1;
			ioe0b_q <= 1'b1;
			ioe1b_q <= 1'b1;
			rdb_q <= 1'b1;
			wrb_q <= 1'b1;
			va_q <= 13'h0000;
			vd_dout_q <= 8'h00;
			vd_oe_q <= 1'b0;
			vce0b_q <= 1'b1;
			vce1b_q <= 1'b1;
			vrdb_q <= 1'b1;
			vwrb_q <= 1'b1;

			// The LCDC keeps fetching its scanline while the CPU bus is idle.
			// Its address is computed once per beat, above, because idle_bus is
			// inlined at every idle state.
			if (lcdc_scan_active_v) begin
				va_q <= lcdc_scan_addr_v;
				vrdb_q <= 1'b0;
				if (!lcc_q[6]) begin
					vce0b_q <= 1'b0;
				end else begin
					vce1b_q <= 1'b0;
				end
			end
		end
	endtask

	task start_uart_tx;
		input [7:0] data;
		reg [11:0] frame_v;
		reg [3:0] bits_v;
		begin
			frame_v = 12'hFFF;
			frame_v[0] = 1'b0;
			frame_v[8:1] = data;
			bits_v = 4'd9;
			if (!urtc_q[2]) begin
				frame_v[9] = uart_parity_bit(data);
				bits_v = 4'd10;
			end
			frame_v[bits_v] = 1'b1;
			bits_v = bits_v + 4'd1;
			if (urtc_q[0]) begin
				frame_v[bits_v] = 1'b1;
				bits_v = bits_v + 4'd1;
			end

			txdb_q <= frame_v[0];
			uart_tx_shift_q <= {1'b1, frame_v[11:1]};
			uart_tx_bits_q <= bits_v - 4'd1;
			uart_tx_div_q <= uart_bit_period_w - 16'd1;
			uart_tx_active_q <= 1'b1;
		end
	endtask

	// Reads that clear a status bit as a side effect. The four entry points
	// differ only in what they are handed: a raw SFR address, a direct address
	// that may or may not be an SFR, a direct address pair, or an already
	// classified bus access.
	task apply_sfr_read_side_effects;
		input [7:0] addr;
		begin
			case (addr)
				8'h2C: urts_q[0] <= 1'b0;
				8'h2D: urts_q[4:2] <= 3'b000;
				default: begin
				end
			endcase
		end
	endtask

	task apply_direct_read_side_effects;
		input [7:0] addr;
		begin
			if (!addr[7] && (addr >= 8'h10)) begin
				apply_sfr_read_side_effects(addr);
			end
		end
	endtask

	task apply_direct_read_word_side_effects;
		input [7:0] addr;
		begin
			apply_direct_read_side_effects(addr);
			apply_direct_read_side_effects(addr + 8'h01);
		end
	endtask

	task apply_read_side_effects;
		input [2:0] kind;
		input [15:0] addr;
		begin
			if (kind == AK_SFR) begin
				apply_sfr_read_side_effects(addr[7:0]);
			end
		end
	endtask

	// Bus access requests.
	//
	// The state machine makes at most one bus access per beat. Call sites only
	// record what they want; apply_bus_request carries it out at the end of the
	// CPU block. That keeps the access classification, the MMU translation and
	// the chip-select decode in one place instead of at all twenty call sites.
	// FFxx is the I/O window; below it the ROM-class and SRAM-class selects
	// split at the top of the 64 KiB map.

	// One fetched operand byte, and the PC steps past it.
	task take_operand_byte;
		input [5:0] next_state;
		begin
			operand0_q <= access_rdata_cheat_w;
			apply_read_side_effects(access_kind_q, access_addr_q);
			pc_q <= pc_q + 16'h0001;
			idle_bus();
			state_q <= next_state;
		end
	endtask

	task drive_ext_chip_select;
		input [15:0] addr;
		begin
			if (addr[15:8] == 8'hFF) begin
				ioe0b_q <= 1'b0;
				ioe1b_q <= 1'b0;
			end else if (addr[15:13] <= 3'b100) begin
				mce0b_q <= 1'b0;
			end else begin
				mce1b_q <= 1'b0;
			end
		end
	endtask

	task begin_read;
		input [15:0] addr;
		begin
			bus_rd_req_v = 1'b1;
			bus_rd_addr_v = addr;
			bus_rd_kind_known_v = 1'b0;
			bus_rd_phys_v = 1'b0;
		end
	endtask

	// The DMA's ROM source read reaches the cartridge bus directly, with a bank
	// number, rather than through one of the CPU's MMU windows.
	task begin_rom_phys_read;
		input [20:0] phys_addr;
		begin
			begin_read(16'h0000);
			bus_rd_kind_v = AK_EXT;
			bus_rd_kind_known_v = 1'b1;
			bus_rd_phys_v = 1'b1;
			bus_rd_phys_addr_v = phys_addr;
		end
	endtask

	// For an access whose class the caller already knows, such as a DMA
	// destination read that is VRAM or external by transfer mode.
	task begin_read_classified;
		input [15:0] addr;
		input [2:0] kind;
		begin
			begin_read(addr);
			bus_rd_kind_v = kind;
			bus_rd_kind_known_v = 1'b1;
		end
	endtask

	// A read that also picks the next state: straight to the sample beat, or
	// through a wait beat when the access class needs extra ones.
	task schedule_read;
		input [15:0] addr;
		input [5:0] wait_state;
		input [5:0] sample_state;
		begin
			begin_read(addr);
			bus_rd_pick_state_v = 1'b1;
			bus_rd_wait_state_v = wait_state;
			bus_rd_sample_state_v = sample_state;
		end
	endtask

	task begin_write;
		input [15:0] addr;
		input [7:0] data;
		begin
			bus_wr_req_v = 1'b1;
			bus_wr_addr_v = addr;
			bus_wr_data_v = data;
		end
	endtask

	// The write goes first so that on a dual-bus DMA beat the read owns
	// access_addr_q and access_kind_q, which is what the sample beat inspects.
	task apply_bus_request;
		reg [2:0] kind_v;
		reg [1:0] wait_states_v;
		begin
			if (bus_wr_req_v) begin
				kind_v = classify_access(bus_wr_addr_v);
				access_addr_q <= bus_wr_addr_v;
				access_kind_q <= kind_v;
				access_wdata_q <= bus_wr_data_v;
				case (kind_v)
					AK_IRAM: begin
						ram_addr_q <= bus_wr_addr_v[9:0];
						ram_wdata_q <= bus_wr_data_v;
					end
					AK_EXT: begin
						a_q <= external_addr(bus_wr_addr_v);
						d_dout_q <= bus_wr_data_v;
						d_oe_q <= 1'b1;
						wrb_q <= 1'b0;
						drive_ext_chip_select(bus_wr_addr_v);
					end
					AK_VRAM: begin
						vd_dout_q <= bus_wr_data_v;
						vd_oe_q <= 1'b1;
						// Release the completed ROM-class request before an optional
						// DMA pipeline helper selects the next byte in this setup beat.
						// The preceding sample beat supplies the distinct request edge.
						a_q <= 21'h000000;
						d_dout_q <= 8'h00;
						d_oe_q <= 1'b0;
						mce0b_q <= 1'b1;
						mce1b_q <= 1'b1;
						ioe0b_q <= 1'b1;
						ioe1b_q <= 1'b1;
						rdb_q <= 1'b1;
						wrb_q <= 1'b1;
						// Override any scanout-owned VRAM strobes from idle_bus() before
						// driving a CPU/DMA write on the shared VRAM bus.
						vce0b_q <= 1'b1;
						vce1b_q <= 1'b1;
						vrdb_q <= 1'b1;
						vwrb_q <= 1'b0;
						va_q <= bus_wr_addr_v[12:0];
						if (!bus_wr_addr_v[14]) begin
							vce0b_q <= 1'b0;
						end else begin
							vce1b_q <= 1'b0;
						end
					end
					default: begin
					end
				endcase
			end

			if (bus_rd_req_v) begin
				kind_v = bus_rd_kind_known_v ? bus_rd_kind_v : classify_access(bus_rd_addr_v);
				access_addr_q <= bus_rd_addr_v;
				access_kind_q <= kind_v;
				access_wdata_q <= 8'h00;
				case (kind_v)
					AK_IRAM: ram_addr_q <= bus_rd_addr_v[9:0];
					AK_IROM: rom_addr_q <= bus_rd_addr_v[11:0];
					AK_EXT: begin
						d_oe_q <= 1'b0;
						rdb_q <= 1'b0;
						if (bus_rd_phys_v) begin
							a_q <= bus_rd_phys_addr_v;
							mce0b_q <= 1'b0;
						end else begin
							a_q <= external_addr(bus_rd_addr_v);
							drive_ext_chip_select(bus_rd_addr_v);
						end
					end
					AK_VRAM: begin
						va_q <= bus_rd_addr_v[12:0];
						vd_oe_q <= 1'b0;
						// Override any scanout-owned VRAM strobes from idle_bus() before
						// selecting the CPU/DMA target page on the shared VRAM bus.
						vce0b_q <= 1'b1;
						vce1b_q <= 1'b1;
						vwrb_q <= 1'b1;
						vrdb_q <= 1'b0;
						if (!bus_rd_addr_v[14]) begin
							vce0b_q <= 1'b0;
						end else begin
							vce1b_q <= 1'b0;
						end
					end
					default: begin
					end
				endcase

				if (bus_rd_pick_state_v) begin
					// The setup beat plus the sample beat already implements the
					// documented 2-cycle baseline for IRAM/VRAM/normal external reads.
					// read_wait_states_kind() therefore only reports extra middle beats
					// beyond that baseline. SDRAM-backed cart ROM latency is modeled
					// entirely by holding the sample beat until rom_read_ready_i is
					// asserted.
					wait_states_v = read_wait_states_kind(kind_v);
					if (wait_states_v != 2'd0) begin
						read_wait_q <= wait_states_v - 2'd1;
						state_q <= bus_rd_wait_state_v;
					end else begin
						state_q <= bus_rd_sample_state_v;
					end
				end
			end

			bus_wr_req_v = 1'b0;
			bus_rd_req_v = 1'b0;
			bus_rd_phys_v = 1'b0;
			bus_rd_pick_state_v = 1'b0;
		end
	endtask

	//========================================================================
	// Direct-address writes
	//========================================================================
	// Queue a direct-address write for this beat. See dwr_count_v.
	task direct_write;
		input [7:0] addr;
		input [7:0] data;
		begin
			case (dwr_count_v)
				2'd0: begin dwr_addr0_v = addr; dwr_data0_v = data; end
				2'd1: begin dwr_addr1_v = addr; dwr_data1_v = data; end
				default: begin dwr_addr2_v = addr; dwr_data2_v = data; end
			endcase
`ifndef SYNTHESIS
			if (dwr_count_v == 2'd3) begin
				$display("FAIL direct_write queue overflow at %0t", $time);
				$finish(1);
			end
`endif
			dwr_count_v = dwr_count_v + 2'd1;
		end
	endtask

	// The one drain, at the end of the beat. The decode below therefore exists
	// three times instead of at every call site.
	task apply_pending_writes;
		begin
			if (dwr_count_v > 2'd0) apply_direct_write(dwr_addr0_v, dwr_data0_v);
			if (dwr_count_v > 2'd1) apply_direct_write(dwr_addr1_v, dwr_data1_v);
			if (dwr_count_v > 2'd2) apply_direct_write(dwr_addr2_v, dwr_data2_v);
			dwr_count_v = 2'd0;
		end
	endtask

	task apply_direct_write;
		input [7:0] addr;
		input [7:0] data;
		begin
			if (addr < 8'h10) begin
				gp_write(addr[3:0], data);
			end else if (addr[7]) begin
				write_iram_shadow(addr, data);
				ram_addr_q <= {2'b00, addr};
				ram_wdata_q <= iram_stored_v;
				ram_wren_q <= 1'b1;
			end else begin
				shadow_sfr_write(addr[6:0], data);
`ifndef SYNTHESIS
				lowmem_q[addr] <= data;
`endif
				case (addr)
					8'h10: begin
						ie0_q <= data & IE0_VALID_MASK;
						shadow_sfr_write(addr[6:0], data & IE0_VALID_MASK);
					end
					8'h11: begin
						ie1_q <= data & IE1_VALID_MASK;
						shadow_sfr_write(addr[6:0], data & IE1_VALID_MASK);
					end
					8'h12: begin
						ir0_q <= data & IR0_VALID_MASK;
						shadow_sfr_write(addr[6:0], data & IR0_VALID_MASK);
						// Direct writes update the visible request latch. Writing zero
						// is also the explicit software clear for any matching private
						// edge that has not been consumed yet.
						if (!data[7]) irq_dma_pending_q <= 1'b0;
						if (!data[6]) irq_tim0_pending_q <= 1'b0;
						if (!data[4]) ext_pending_q <= 1'b0;
						if (!data[3]) irq_uart_pending_q <= 1'b0;
						if (!data[0]) irq_lcdc_pending_q <= 1'b0;
					end
					8'h13: begin
						ir1_q <= data & IR1_VALID_MASK;
						shadow_sfr_write(addr[6:0], data & IR1_VALID_MASK);
						// See IR0 direct-write note above.
						if (!data[6]) irq_tim1_pending_q <= 1'b0;
						if (!data[4]) irq_clk_pending_q <= 1'b0;
						if (!data[2]) irq_pio_pending_q <= 1'b0;
					end
					8'h14: p0_q <= data;
					8'h15: begin
						p1_q <= data;
						pio_scan_update_q <= 1'b1;
					end
					8'h16: begin
						p2_q <= data;
						pio_scan_update_q <= 1'b1;
					end
					8'h17: p3_q <= data;
					8'h19: begin
						sys_q <= data;
						shadow_sfr_write(addr[6:0], data);
						if (!data[6]) sp_q[15:8] <= 8'h00;
					end
					8'h1A: ckc_q <= data;
					8'h1C: if (sys_q[6]) sp_q[15:8] <= data;
					8'h1D: sp_q[7:0] <= data;
					8'h1E: begin
						ps0_q <= data;
`ifndef SYNTHESIS
						refresh_gp_mirror(data[7:3]);
`endif
					end
					8'h1F: ps1_q <= data;
					8'h20: p0c_q <= data;
					8'h21: p1c_q <= data;
					8'h22: p2c_q <= data;
					8'h23: p3c_q <= data;
					8'h24: begin
						mmu0_q <= data;
						mmu0_en_q <= 1'b1;
					end
					8'h25: begin
						mmu1_q <= data;
						mmu1_resolved_q <= data;
					end
					8'h26: begin
						mmu2_q <= data;
						mmu2_resolved_q <= data;
					end
					8'h27: begin
						mmu3_q <= data;
						mmu3_resolved_q <= data;
					end
					8'h28: begin
						mmu4_q <= data;
						mmu4_resolved_q <= data;
					end
					8'h30: begin
						lcc_q <= data;
						shadow_sfr_write(addr[6:0], data);
					end
					8'h31: begin
						lch_q <= data;
						shadow_sfr_write(addr[6:0], data);
					end
					8'h32: begin
						lcv_q <= {1'b0, data[6:0]};
						shadow_sfr_write(addr[6:0], {1'b0, data[6:0]});
					end
					8'h34: begin
						dmc_q <= data;
						if (!data[7]) begin
							dma_active_q <= 1'b0;
						end
					end
					8'h35: dmx1_q <= data;
					8'h36: dmy1_q <= data;
					8'h37: dmdx_q <= data;
					8'h38: dmdy_q <= data;
					8'h39: dmx2_q <= data;
					8'h3A: dmy2_q <= data;
					8'h3B: dmpl_q <= data;
					8'h3C: begin
						dmbr_q <= data;
					end
					8'h3D: begin
						dmvp_q <= data;
					end
					8'h2B: begin
						urtt_q <= data;
						shadow_sfr_write(addr[6:0], data);
						urts_q[1] <= 1'b0;
						if (urtc_q[4] && !uart_tx_active_q) begin
							start_uart_tx(data);
						end
					end
					8'h2D: begin
						urts_q[7:6] <= data[7:6];
					end
					8'h2E: begin
						urtc_q <= data;
						shadow_sfr_write(addr[6:0], data);
						if (!data[4]) begin
							uart_tx_active_q <= 1'b0;
							uart_tx_bits_q <= 4'd0;
							uart_tx_div_q <= 16'h0000;
							uart_tx_shift_q <= 12'hFFF;
							txdb_q <= 1'b1;
							urts_q[1] <= 1'b1;
						end else if (!uart_tx_active_q && !urts_q[1]) begin
							start_uart_tx(urtt_q);
						end
						if (!data[3]) begin
							uart_rx_active_q <= 1'b0;
							uart_rx_div_q <= 16'h0000;
							reset_uart_rx(1'b0);
							urts_q[5] <= 1'b0;
						end
					end
					8'h40: begin
						sgc_q <= data;
					end
					8'h42: begin
						sg0l_q <= {3'b000, data[4:0]};
						shadow_sfr_write(addr[6:0], {3'b000, data[4:0]});
					end
					8'h44: begin
						sg1l_q <= {3'b000, data[4:0]};
						shadow_sfr_write(addr[6:0], {3'b000, data[4:0]});
					end
					8'h46: begin
						sg0th_q <= {4'h0, data[3:0]};
						shadow_sfr_write(addr[6:0], {4'h0, data[3:0]});
						sg0t_q[15:12] <= 4'h0;
						sg0t_q[11:8] <= data[3:0];
					end
					8'h47: begin
						sg0t_q[7:0] <= data;
						// The low byte completes the software's usual MOVW SG0T write.
						// The sound block observes the write, but does not phase-reset a
						// live 32-step burst just because software refreshed the period.
					end
					8'h48: begin
						sg1th_q <= {4'h0, data[3:0]};
						shadow_sfr_write(addr[6:0], {4'h0, data[3:0]});
						sg1t_q[15:12] <= 4'h0;
						sg1t_q[11:8] <= data[3:0];
					end
					8'h49: begin
						sg1t_q[7:0] <= data;
					end
					8'h4A: begin
						sg2l_q <= {3'b000, data[4:0]};
						shadow_sfr_write(addr[6:0], {3'b000, data[4:0]});
					end
					8'h4C: begin
						sg2th_q <= {4'h0, data[3:0]};
						shadow_sfr_write(addr[6:0], {4'h0, data[3:0]});
						sg2t_q[15:12] <= 4'h0;
						sg2t_q[11:8] <= data[3:0];
					end
					8'h4D: begin
						sg2t_q[7:0] <= data;
					end
					8'h4E: begin
						sgda_q <= data;
						sgda_write_strobe_q <= 1'b1;
					end
					// GUESS: although the datasheet marks TMxC[6:3] as zero,
					// Game.com software/MAME treat the SFR storage byte as raw.
					// Active timer decode still consumes only bit 7 and bits 2:0.
					8'h50: begin
						tm0c_q <= data;
						tm0c_write_seq_q <= tm0c_write_seq_q + 2'b01;
						shadow_sfr_write(addr[6:0], data);
					end
					8'h51: begin
						tm0_reload_q <= data;
						tm0_reload_write_seq_q <= tm0_reload_write_seq_q + 2'b01;
						shadow_sfr_write(addr[6:0], data);
					end
					8'h52: begin
						tm1c_q <= data;
						tm1c_write_seq_q <= tm1c_write_seq_q + 2'b01;
						shadow_sfr_write(addr[6:0], data);
					end
					8'h53: begin
						tm1_reload_q <= data;
						tm1_reload_write_seq_q <= tm1_reload_write_seq_q + 2'b01;
						shadow_sfr_write(addr[6:0], data);
					end
					8'h54: begin
						clkt_run_q <= data[7];
						clkt_minute_q <= data[6];
						clkt_write_seq_q <= clkt_write_seq_q + 2'b01;
						shadow_sfr_write(addr[6:0], {data[7], data[6], clkt_count_q});
					end
					8'h5F: begin
						wdtc_q <= data;
						shadow_sfr_write(addr[6:0], data);
						if (!data[7] || data[3]) begin
							wdt_q <= 8'h00;
							shadow_sfr_write(7'h5E, 8'h00);
							wdt_div_q <= wdt_tick_period(data[2:0]) - 24'd1;
						end else if (!wdtc_q[7] || (wdtc_q[2:0] != data[2:0])) begin
							wdt_div_q <= wdt_tick_period(data[2:0]) - 24'd1;
						end
					end
					8'h60, 8'h61, 8'h62, 8'h63, 8'h64, 8'h65, 8'h66, 8'h67,
					8'h68, 8'h69, 8'h6A, 8'h6B, 8'h6C, 8'h6D, 8'h6E, 8'h6F: begin
						if (!sgc_q[0]) begin
							sg0w_q[addr[3:0]] <= data;
						end else begin
							shadow_sfr_write(addr[6:0], sg0w_q[addr[3:0]]);
						end
					end
					8'h70, 8'h71, 8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77,
					8'h78, 8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7E, 8'h7F: begin
						if (!sgc_q[1]) begin
							sg1w_q[addr[3:0]] <= data;
						end else begin
							shadow_sfr_write(addr[6:0], sg1w_q[addr[3:0]]);
						end
					end
					default: begin
						write_sfr_hole(addr, data);
					end
				endcase
			end
		end
	endtask

	task direct_write_word;
		input [7:0] addr;
		input [15:0] data;
		begin
			// Do not mask bit 0: Sharp documents odd RR results as unreliable,
			// rather than defining an even-address alias.
			direct_write(addr, data[15:8]);
			direct_write(addr + 8'h01, data[7:0]);
		end
	endtask

	//========================================================================
	// Effective-address generation
	//========================================================================
	// Four addressing shapes reach memory, and every instruction that uses one
	// requests exactly one per beat. The request is recorded here and carried
	// out once at the end of the CPU block, so the address arithmetic exists
	// in one place rather than at all sixteen call sites.
	//
	// The register-indirect pair modes come from desc[7:6], with desc[2:0]
	// naming the pair. RMW steps by one byte and SMW by two:
	//
	//   00  @RRn        01  @RRn+
	//   10  @RRn + d16  11  @-RRn
	task request_addr;
		input [2:0] kind;
		input [7:0] desc;
		input [15:0] imm16;
		begin
			agen_kind_v = kind;
			agen_desc_v = desc;
			agen_imm_v = imm16;
		end
	endtask

	task apply_addr_request;
		reg [3:0] base_v;
		reg [15:0] base_addr_v;
		reg [15:0] step_v;
		reg [15:0] addr_v;
		begin
			base_v = pair_base(agen_desc_v[2:0]);
			base_addr_v = gp_read_word(base_v);
			step_v = (agen_kind_v == AGEN_SMW) ? 16'h0002 : 16'h0001;
			addr_v = agen_imm_v;

			case (agen_kind_v)
				AGEN_RMW, AGEN_SMW: begin
					case (agen_desc_v[7:6])
						2'b01: begin
							addr_v = base_addr_v;
							direct_write_word({4'b0000, base_v}, base_addr_v + step_v);
						end
						2'b10: if (agen_desc_v[2:0] != 3'b000) addr_v = agen_imm_v + base_addr_v;
						2'b11: begin
							addr_v = base_addr_v - step_v;
							direct_write_word({4'b0000, base_v}, addr_v);
						end
						default: addr_v = base_addr_v;
					endcase
				end

				// A 16-bit displacement, optionally indexed by a single register.
				AGEN_ARG2: if (agen_desc_v[5:3] != 3'b000)
					addr_v = agen_imm_v + {8'h00, gp_read({1'b0, agen_desc_v[5:3]})};

				// An 8-bit displacement. Without an index register it names the
				// FFxx page; with one it stays in the zero page.
				AGEN_RI: addr_v = (agen_desc_v[5:3] != 3'b000) ?
					{8'h00, agen_imm_v[7:0] + gp_read({1'b0, agen_desc_v[5:3]})} :
					{8'hFF, agen_imm_v[7:0]};

				default: begin
				end
			endcase

			if (agen_kind_v != AGEN_NONE) eff_addr_q <= addr_v;
			agen_kind_v = AGEN_NONE;
		end
	endtask

	function [7:0] source_after_rmb_update;
		input [7:0] desc;
		reg [3:0] src_idx_v;
		reg [3:0] ptr_idx_v;
		reg [7:0] ptr_val_v;
		begin
			src_idx_v = {1'b0, desc[5:3]};
			ptr_idx_v = {1'b0, desc[2:0]};
			ptr_val_v = 8'h00;
			source_after_rmb_update = gp_read(src_idx_v);
			if (src_idx_v == ptr_idx_v) begin
				ptr_val_v = gp_read(ptr_idx_v);
				case (desc[7:6])
					2'b01: source_after_rmb_update = ptr_val_v + 8'h01;
					2'b11: source_after_rmb_update = ptr_val_v - 8'h01;
					default: begin
					end
				endcase
			end
		end
	endfunction

	function [7:0] source_after_rmw_update;
		input [7:0] desc;
		reg [3:0] src_idx_v;
		reg [3:0] ptr_idx_v;
		reg [15:0] ptr_val_v;
		begin
			src_idx_v = {1'b0, desc[5:3]};
			ptr_idx_v = pair_base(desc[2:0]);
			ptr_val_v = gp_read_word(ptr_idx_v);
			case (desc[7:6])
				2'b01: ptr_val_v = ptr_val_v + 16'h0001;
				2'b11: ptr_val_v = ptr_val_v - 16'h0001;
				default: begin
				end
			endcase
			if (src_idx_v == ptr_idx_v) begin
				source_after_rmw_update = ptr_val_v[15:8];
			end else if (src_idx_v == (ptr_idx_v + 4'd1)) begin
				source_after_rmw_update = ptr_val_v[7:0];
			end else begin
				source_after_rmw_update = gp_read(src_idx_v);
			end
		end
	endfunction

	function [15:0] source_after_smw_update;
		input [7:0] desc;
		reg [3:0] src_idx_v;
		reg [3:0] ptr_idx_v;
		begin
			src_idx_v = pair_base(desc[5:3]);
			ptr_idx_v = pair_base(desc[2:0]);
			source_after_smw_update = gp_read_word(src_idx_v);
			if (src_idx_v == ptr_idx_v) begin
				case (desc[7:6])
					2'b01: source_after_smw_update = source_after_smw_update + 16'h0002;
					2'b11: source_after_smw_update = source_after_smw_update - 16'h0002;
					default: begin
					end
				endcase
			end
		end
	endfunction

	localparam [3:0] ALU_CMP = 4'h0;
	localparam [3:0] ALU_ADD = 4'h1;
	localparam [3:0] ALU_SUB = 4'h2;
	localparam [3:0] ALU_ADC = 4'h3;
	localparam [3:0] ALU_SBC = 4'h4;
	localparam [3:0] ALU_AND = 4'h5;
	localparam [3:0] ALU_OR  = 4'h6;
	localparam [3:0] ALU_XOR = 4'h7;
	localparam [3:0] ALU_MOV = 4'h8;

	//========================================================================
	// ALU and instruction execution
	//========================================================================
	// The byte ALU appears in four encodings that differ only in where the
	// operands live: compact register (10-17), memory (20-27 and 30-37), fixed
	// direct (40-48) and direct immediate (50-58). The opcode's low nibble is
	// the operation in all four, and the constants above name it.
	//
	// CMP is a SUB whose result is thrown away, so callers store only when
	// store_o is set. MOV touches no flag.
	task byte_alu;
		input [3:0] op_in;
		input [7:0] lhs_v;
		input [7:0] rhs_v;
		output [7:0] res_o;
		output store_o;
		reg [8:0] sum_v;
		reg [7:0] ps_v;
		reg carry_in_v;
		reg subtract_v;
		begin
			carry_in_v = ((op_in == ALU_ADC) || (op_in == ALU_SBC)) && ps1_q[7];
			subtract_v = (op_in == ALU_CMP) || (op_in == ALU_SUB) || (op_in == ALU_SBC);
			sum_v = subtract_v ?
				({1'b0, lhs_v} - {1'b0, rhs_v} - {8'h00, carry_in_v}) :
				({1'b0, lhs_v} + {1'b0, rhs_v} + {8'h00, carry_in_v});

			case (op_in)
				ALU_AND: res_o = lhs_v & rhs_v;
				ALU_OR:  res_o = lhs_v | rhs_v;
				ALU_XOR: res_o = lhs_v ^ rhs_v;
				ALU_MOV: res_o = rhs_v;
				default: res_o = sum_v[7:0];
			endcase
			store_o = (op_in != ALU_CMP) && (op_in <= ALU_MOV);

			// MOV reports nothing, and neither does an opcode outside the set.
			if (op_in < ALU_MOV) begin
				if (op_in <= ALU_SBC) begin
					// Subtraction reports its borrow through D. CMP alone keeps
					// the half-carry and decimal flags it was handed.
					ps_v = (op_in == ALU_CMP) ?
						(ps1_q & (FLAG_B | FLAG_I | FLAG_H | FLAG_D)) :
						(ps1_q & (FLAG_B | FLAG_I));
					if (subtract_v && (op_in != ALU_CMP)) ps_v = ps_v | FLAG_D;
					if (sum_v[8]) ps_v = ps_v | FLAG_C;
					// Signed overflow: the operands must differ in sign for a
					// subtract and agree for an add, and the result must differ
					// in sign from the left operand.
					if ((((subtract_v ? (lhs_v ^ rhs_v) : ~(lhs_v ^ rhs_v)) &
						(lhs_v ^ sum_v[7:0])) & 8'h80) != 8'h00) ps_v = ps_v | FLAG_V;
					if ((op_in != ALU_CMP) &&
						(((lhs_v ^ rhs_v ^ sum_v[7:0]) & 8'h10) != 8'h00)) ps_v = ps_v | FLAG_H;
				end else begin
					ps_v = ps1_q & (FLAG_B | FLAG_C | FLAG_I | FLAG_H | FLAG_D);
				end
				if (res_o == 8'h00) ps_v = ps_v | FLAG_Z;
				if (res_o[7]) ps_v = ps_v | FLAG_S;
				ps1_q <= ps_v;
			end
		end
	endtask

	// CMP without a destination, for the two classes that only want the flags.
	// CMP sets flags and keeps no result, so the two outputs are declared only
	// to be discarded.
	task flags_cmp8;
		input [7:0] lhs;
		input [7:0] rhs;
		reg [7:0] res_v;
		reg store_v;
		begin
			byte_alu(ALU_CMP, lhs, rhs, res_v, store_v);
		end
	endtask

	// The 20-27H and 30-37H forms always name a general-purpose register as
	// their destination, so this one takes a register index, not an address.
	task exec_mem_byte_alu;
		input [7:0] opcode_in;
		input [3:0] dst_idx;
		input [7:0] rhs_v;
		reg [7:0] res_v;
		reg store_v;
		begin
			byte_alu(opcode_in[3:0], gp_read(dst_idx), rhs_v, res_v, store_v);
			if (store_v) begin
				alu_res_q <= res_v;
				gp_write(dst_idx, res_v);
			end
		end
	endtask

	task exec_direct_imm_op;
		input [7:0] opcode_in;
		input [7:0] addr;
		input [7:0] lhs_v;
		input [7:0] imm_v;
		reg [7:0] res_v;
		reg store_v;
		begin
			byte_alu(opcode_in[3:0], lhs_v, imm_v, res_v, store_v);
			apply_direct_read_side_effects(addr);
			if (store_v) begin
				alu_res_q <= res_v;
				direct_write(addr, res_v);
			end
		end
	endtask

	task exec_direct_unary_op;
		input [7:0] opcode_in;
		input [7:0] addr;
		input [15:0] operand;
		reg [7:0] res_v;
		begin
			// 18H/19H are the only 16-bit forms in this group.
			if (opcode_in == 8'h18 || opcode_in == 8'h19) begin
				exec_unary16(opcode_in == 8'h19, addr, operand);
				apply_direct_read_word_side_effects(addr);
			end else begin
				unary_alu(direct_unary_op(opcode_in), operand[15:8], res_v);
				apply_direct_read_side_effects(addr);
				alu_res_q <= res_v;
				direct_write(addr, res_v);
			end
		end
	endtask

	// Opcodes 01H-0DH in encoding order.
	function [3:0] direct_unary_op;
		input [7:0] opcode_in;
		begin
			case (opcode_in)
				8'h01: direct_unary_op = UN_NEG;
				8'h02: direct_unary_op = UN_COM;
				8'h03: direct_unary_op = UN_RR;
				8'h04: direct_unary_op = UN_RL;
				8'h05: direct_unary_op = UN_RRC;
				8'h06: direct_unary_op = UN_RLC;
				8'h07: direct_unary_op = UN_SRL;
				8'h08: direct_unary_op = UN_INC;
				8'h09: direct_unary_op = UN_DEC;
				8'h0A: direct_unary_op = UN_SRA;
				8'h0B: direct_unary_op = UN_SLL;
				8'h0C: direct_unary_op = UN_DA;
				default: direct_unary_op = UN_SWAP;
			endcase
		end
	endfunction

	// The indirect forms pack the same operations into two opcodes, 1AH and
	// 1BH, with a three-bit selector taken from the operand byte.
	function [3:0] indirect_unary_op;
		input [7:0] opcode_in;
		input [2:0] op_sel;
		begin
			if (opcode_in == 8'h1A) begin
				case (op_sel)
					3'b001: indirect_unary_op = UN_NEG;
					3'b010: indirect_unary_op = UN_COM;
					3'b011: indirect_unary_op = UN_RR;
					3'b100: indirect_unary_op = UN_RL;
					3'b101: indirect_unary_op = UN_RRC;
					3'b110: indirect_unary_op = UN_RLC;
					default: indirect_unary_op = UN_SRL;
				endcase
			end else begin
				case (op_sel)
					3'b000: indirect_unary_op = UN_INC;
					3'b001: indirect_unary_op = UN_DEC;
					3'b010: indirect_unary_op = UN_SRA;
					3'b011: indirect_unary_op = UN_SLL;
					3'b100: indirect_unary_op = UN_DA;
					default: indirect_unary_op = UN_SWAP;
				endcase
			end
		end
	endfunction

	task exec_indirect_unary_op;
		input [7:0] opcode_in;
		input [2:0] op_sel;
		input [7:0] addr;
		input [7:0] operand;
		reg [7:0] res_v;
		begin
			// 1AH selector 0 is CLR, which does not read the old destination.
			if ((opcode_in == 8'h1A) && (op_sel == 3'b000)) begin
				direct_write(addr, 8'h00);
			end else begin
				unary_alu(indirect_unary_op(opcode_in, op_sel), operand, res_v);
				apply_direct_read_side_effects(addr);
				alu_res_q <= res_v;
				direct_write(addr, res_v);
			end
		end
	endtask

	function [3:0] indirect_unary_extra_cycles;
		input [7:0] opcode_in;
		input [2:0] op_sel;
		begin
			if (opcode_in == 8'h1A) begin
				case (op_sel)
					3'b001: indirect_unary_extra_cycles = 4'd4;
					3'b111: indirect_unary_extra_cycles = 4'd2;
					default: indirect_unary_extra_cycles = 4'd3;
				endcase
			end else begin
				case (op_sel)
					3'b010, 3'b011: indirect_unary_extra_cycles = 4'd2;
					// Software behavior confirms SWAP semantics, but not MAME's
					// anomalous premium; keep it with the ordinary unary class.
					default: indirect_unary_extra_cycles = 4'd3;
				endcase
			end
		end
	endfunction

	task exec_word_rr_op;
		input [7:0] opcode_in;
		input [7:0] dst_addr;
		input [7:0] src_addr;
		input [15:0] dst_v;
		input [15:0] src_v;
		reg [15:0] rhs_v;
		reg [15:0] res_v;
		reg store_v;
		begin
			rhs_v = word_after_sfr_reads(
				src_addr,
				src_v,
				(dst_addr == 8'h2C) || ((dst_addr + 8'h01) == 8'h2C),
				(dst_addr == 8'h2D) || ((dst_addr + 8'h01) == 8'h2D)
			);
			apply_direct_read_word_side_effects(dst_addr);
			apply_direct_read_word_side_effects(src_addr);
			word_alu(opcode_in[2:0], dst_v, rhs_v, res_v, store_v);
			if (store_v) direct_write_word(dst_addr, res_v);
		end
	endtask

	task exec_word_imm_op;
		input [7:0] opcode_in;
		input [7:0] dst_addr;
		input [15:0] dst_v;
		input [15:0] imm_v;
		reg [15:0] res_v;
		reg store_v;
		begin
			word_alu(opcode_in[2:0], dst_v, imm_v, res_v, store_v);
			apply_direct_read_word_side_effects(dst_addr);
			if (store_v) direct_write_word(dst_addr, res_v);
		end
	endtask

	task exec_fixed_byte_alu;
		input [7:0] opcode_in;
		input [7:0] dst_addr;
		input [7:0] src_addr;
		input [7:0] dst_v;
		input [7:0] src_v;
		reg [7:0] lhs_v;
		reg [7:0] rhs_v;
		reg [7:0] res_v;
		reg store_v;
		reg dst_read_v;
		begin
			// MOV (48H) only writes the destination, so it does not read it and
			// cannot take the destination's read side effects.
			dst_read_v = opcode_in <= 8'h47;
			lhs_v = dst_read_v ? dst_v : 8'h00;
			rhs_v = after_sfr_reads(
				src_addr,
				src_v,
				dst_read_v && (dst_addr == 8'h2C),
				dst_read_v && (dst_addr == 8'h2D)
			);
			if (dst_read_v) apply_direct_read_side_effects(dst_addr);
			apply_direct_read_side_effects(src_addr);
			byte_alu(opcode_in[3:0], lhs_v, rhs_v, res_v, store_v);
			if (store_v) begin
				mem_byte_q <= res_v;
				direct_write(dst_addr, res_v);
			end
		end
	endtask

	task exec_exts_direct_op;
		input [7:0] addr;
		input [7:0] msb_v;
		begin
			// Sharp/Sacred define sign extension from an even pair's low byte;
			// Batman's coordinate routines corroborate this exact dataflow.
			apply_direct_read_side_effects(addr + 8'h01);
			if (msb_v[7]) begin
				direct_write_word(addr, {8'hFF, msb_v});
			end else begin
				direct_write_word(addr, {8'h00, msb_v});
			end
		end
	endtask

	task exec_btst_direct_op;
		input [7:0] addr;
		input [7:0] mask_v;
		input [7:0] val_v;
		reg [7:0] ps_v;
		begin
			ps_v = ps1_q & ~(FLAG_Z | FLAG_V);
			apply_direct_read_side_effects(addr);
			if ((val_v & mask_v) == 8'h00) ps_v = ps_v | FLAG_Z;
			ps1_q <= ps_v;
		end
	endtask

	// The thirteen single-operand byte operations, in one place. An arm sets
	// the result and the flags it computes; FLAG_D, FLAG_H, FLAG_B and FLAG_I
	// are never touched, and Z and S follow from the result. Callers store the
	// result themselves, so an instruction writes its destination once.
	task unary_alu;
		input [3:0] op_in;
		input [7:0] val;
		output [7:0] res_o;
		reg [7:0] ps_v;
		reg [8:0] da_v;
		reg sign_v;
		begin
			ps_v = ps1_q & (FLAG_D | FLAG_H | FLAG_B | FLAG_I);
			sign_v = 1'b1;
			case (op_in)
				UN_NEG: begin
					res_o = -val;
					if (res_o == 8'h00) ps_v = ps_v | FLAG_C;
					if (res_o == 8'h80) ps_v = ps_v | FLAG_V;
				end

				UN_COM: begin
					res_o = ~val;
					ps_v = ps_v | (ps1_q & FLAG_C);
				end

				UN_RR: begin
					res_o = {val[0], val[7:1]};
					if (val[0]) ps_v = ps_v | FLAG_C;
					if ((((val ^ res_o) & 8'h80) != 8'h00) && !res_o[7]) ps_v = ps_v | FLAG_V;
				end

				UN_RL: begin
					res_o = {val[6:0], val[7]};
					if (val[7]) ps_v = ps_v | FLAG_C;
					if (((val ^ res_o) & 8'h80) != 8'h00) ps_v = ps_v | FLAG_V;
				end

				UN_RRC: begin
					res_o = {ps1_q[7], val[7:1]};
					if (val[0]) ps_v = ps_v | FLAG_C;
					if ((((val ^ res_o) & 8'h80) != 8'h00) && !res_o[7]) ps_v = ps_v | FLAG_V;
				end

				UN_RLC: begin
					res_o = {val[6:0], ps1_q[7]};
					if (val[7]) ps_v = ps_v | FLAG_C;
					if (((val ^ res_o) & 8'h80) != 8'h00) ps_v = ps_v | FLAG_V;
				end

				// The only shift that does not report S.
				UN_SRL: begin
					res_o = {1'b0, val[7:1]};
					if (val[0]) ps_v = ps_v | FLAG_C;
					sign_v = 1'b0;
				end

				UN_INC: begin
					res_o = val + 8'h01;
					ps_v = ps_v | (ps1_q & FLAG_C);
					if ((((val ^ res_o) & 8'h80) != 8'h00) && !val[7]) ps_v = ps_v | FLAG_V;
				end

				UN_DEC: begin
					res_o = val - 8'h01;
					ps_v = ps_v | (ps1_q & FLAG_C);
					if ((((val ^ res_o) & 8'h80) != 8'h00) && !res_o[7]) ps_v = ps_v | FLAG_V;
				end

				UN_SRA: begin
					res_o = {val[7], val[7:1]};
					if (val[0]) ps_v = ps_v | FLAG_C;
				end

				UN_SLL: begin
					res_o = {val[6:0], 1'b0};
					if (val[7]) ps_v = ps_v | FLAG_C;
				end

				// Decimal adjust carries its own flag rules end to end: it starts
				// from the whole incoming word and only ever adds carry.
				UN_DA: begin
					da_v = {1'b0, val};
					ps_v = ps1_q;
					if (ps1_q[3]) begin
						if (ps1_q[7]) begin
							da_v = da_v + (ps1_q[2] ? 9'h09A : 9'h0A0);
						end else if (ps1_q[2]) begin
							da_v = da_v + 9'h0FA;
						end
					end else if (ps1_q[7]) begin
						if (ps1_q[2] || (da_v[3:0] >= 4'd10)) begin
							da_v = da_v + 9'h066;
						end else begin
							da_v = da_v + 9'h060;
						end
					end else if (ps1_q[2]) begin
						if (da_v[7:4] < 4'hA) begin
							da_v = da_v + 9'h006;
						end else begin
							da_v = da_v + 9'h066;
							ps_v = ps_v | FLAG_C;
						end
					end else if (da_v[3:0] < 4'd10) begin
						if (da_v[7:4] >= 4'hA) begin
							da_v = da_v + 9'h060;
							ps_v = ps_v | FLAG_C;
						end
					end else if (da_v[7:4] < 4'h9) begin
						da_v = da_v + 9'h006;
					end else begin
						da_v = da_v + 9'h066;
						ps_v = ps_v | FLAG_C;
					end
					ps_v = ps_v & ~(FLAG_Z | FLAG_S);
					res_o = da_v[7:0];
				end

				// SWAP leaves every flag alone.
				default: begin
					res_o = {val[3:0], val[7:4]};
					ps_v = ps1_q;
					sign_v = 1'b0;
				end
			endcase

			if (op_in != UN_SWAP) begin
				if (res_o == 8'h00) ps_v = ps_v | FLAG_Z;
				if (sign_v && res_o[7]) ps_v = ps_v | FLAG_S;
				ps1_q <= ps_v;
			end
		end
	endtask

	// INC/DEC on a register pair. The overflow test looks at the operand for
	// INC and at the result for DEC, which is the only place the two differ.
	task exec_unary16;
		input is_dec;
		input [7:0] addr;
		input [15:0] val;
		reg [15:0] res_v;
		reg [7:0] ps_v;
		begin
			res_v = is_dec ? (val - 16'h0001) : (val + 16'h0001);
			ps_v = ps1_q & (FLAG_C | FLAG_D | FLAG_H | FLAG_B | FLAG_I);
			if (res_v == 16'h0000) ps_v = ps_v | FLAG_Z;
			if (res_v[15]) ps_v = ps_v | FLAG_S;
			if ((((val ^ res_v) & 16'h8000) != 16'h0000) &&
				!(is_dec ? res_v[15] : val[15])) ps_v = ps_v | FLAG_V;
			alu_res_q <= res_v[7:0];
			direct_write_word(addr, res_v);
			ps1_q <= ps_v;
		end
	endtask

	// The word ALU is the byte ALU's first eight operations widened to a
	// register pair, in two encodings: register-register (60-67) and immediate
	// (68-6F). Bits [2:0] of the opcode are the operation in both, and the
	// ALU_* constants name it.
	task word_alu;
		input [2:0] op_in;
		input [15:0] lhs_v;
		input [15:0] rhs_v;
		output [15:0] res_o;
		output store_o;
		reg [16:0] sum_v;
		reg [7:0] ps_v;
		reg carry_in_v;
		reg subtract_v;
		begin
			carry_in_v = ((op_in == ALU_ADC[2:0]) || (op_in == ALU_SBC[2:0])) && ps1_q[7];
			subtract_v = (op_in == ALU_CMP[2:0]) || (op_in == ALU_SUB[2:0]) ||
				(op_in == ALU_SBC[2:0]);
			sum_v = subtract_v ?
				({1'b0, lhs_v} - {1'b0, rhs_v} - {16'h0000, carry_in_v}) :
				({1'b0, lhs_v} + {1'b0, rhs_v} + {16'h0000, carry_in_v});

			case (op_in)
				ALU_AND[2:0]: res_o = lhs_v & rhs_v;
				ALU_OR[2:0]:  res_o = lhs_v | rhs_v;
				ALU_XOR[2:0]: res_o = lhs_v ^ rhs_v;
				default: res_o = sum_v[15:0];
			endcase
			store_o = op_in != ALU_CMP[2:0];

			if (op_in <= ALU_SBC[2:0]) begin
				ps_v = (op_in == ALU_CMP[2:0]) ?
					(ps1_q & (FLAG_B | FLAG_I | FLAG_H | FLAG_D)) :
					(ps1_q & (FLAG_B | FLAG_I));
				if (subtract_v && (op_in != ALU_CMP[2:0])) ps_v = ps_v | FLAG_D;
				if (sum_v[16]) ps_v = ps_v | FLAG_C;
				if ((((subtract_v ? (lhs_v ^ rhs_v) : ~(lhs_v ^ rhs_v)) &
					(lhs_v ^ sum_v[15:0])) & 16'h8000) != 16'h0000) ps_v = ps_v | FLAG_V;
				if ((op_in != ALU_CMP[2:0]) &&
					(((lhs_v ^ rhs_v ^ sum_v[15:0]) & 16'h0010) != 16'h0000)) ps_v = ps_v | FLAG_H;
				if (res_o == 16'h0000) ps_v = ps_v | FLAG_Z;
				if (res_o[15]) ps_v = ps_v | FLAG_S;
			end else begin
				// The 16-bit logic ops report Z only; S is left as it was.
				ps_v = ps1_q & (FLAG_C | FLAG_S | FLAG_B | FLAG_I | FLAG_H | FLAG_D);
				if (res_o == 16'h0000) ps_v = ps_v | FLAG_Z;
			end
			ps1_q <= ps_v;
		end
	endtask

	//========================================================================
	// Interrupts and DMA sequencing
	//========================================================================
	task start_interrupt;
		input [15:0] vector_addr;
		input [3:0] source;
		begin
			vector_addr_q <= vector_addr;
			halted_q <= 1'b0;
			stopped_q <= 1'b0;
			irq_resume_defer_q <= 1'b0;
			// Interrupt entry consumes the private edge latch, not the visible
			// IR0/IR1 status bit. Software clears the visible latch explicitly.
			clear_pending_irq(source);
			state_q <= ST_INT_PUSH0_SETUP;
		end
	endtask

	task clear_pending_irq;
		input [3:0] source;
		begin
			case (source)
				IRQ_DMA: begin
					irq_dma_pending_q <= 1'b0;
				end
				IRQ_TIM0: begin
					irq_tim0_pending_q <= 1'b0;
				end
				IRQ_EXT: begin
					ext_pending_q <= 1'b0;
				end
				IRQ_UART: begin
					irq_uart_pending_q <= 1'b0;
				end
				IRQ_LCDC: begin
					irq_lcdc_pending_q <= 1'b0;
				end
				IRQ_TIM1: begin
					irq_tim1_pending_q <= 1'b0;
				end
				IRQ_CLK: begin
					irq_clk_pending_q <= 1'b0;
				end
				IRQ_PIO: begin
					irq_pio_pending_q <= 1'b0;
				end
				default: begin end
			endcase
		end
	endtask

	task schedule_next_dma_step;
		reg [23:0] period_v;
		begin
			period_v = lcdc_dma_tick_period(lcc_q[3:1]);
			if (period_v <= 24'd1) begin
				lcdc_dma_div_q <= 24'd0;
				state_q <= ST_DMA_READ_SETUP;
			end else begin
				lcdc_dma_div_q <= period_v - 24'd2;
				state_q <= ST_DMA_PACE;
			end
		end
	endtask

	// Read one source byte of the running transfer. Mode, VRAM page and ROM
	// bank always come from the transfer; only the address varies.
	task begin_dma_source_read;
		input [13:0] src_addr_i;
		reg [1:0] mode_i;
		reg [7:0] dmvp_i;
		reg [7:0] dmbr_i;
		begin
			mode_i = dma_mode_q;
			dmvp_i = dma_dmvp_q;
			dmbr_i = dma_dmbr_q;
			if ((mode_i == 2'b00) || (mode_i == 2'b11)) begin
				// DMVP prose defines bit 0 as the source VRAM page and bit 1 as
				// the destination page. Real hardware `dma_lcdc_visual_test` shows
				// DMVP=02 copying from page A to a displayed page-B destination.
				begin_read(dma_vram_byte_cpu_addr(dmvp_i[0], src_addr_i[12:0]));
			end else if (mode_i == 2'b01) begin
				// ROM mode puts a physical ROM address straight onto the board
				// bus instead of going through the CPU's MMU windows, and uses
				// the normal setup/sample transaction model. The sample states
				// hold a cartridge read until the Pocket memory adapter is ready.
				begin_rom_phys_read({dmbr_i[6:0], src_addr_i});
			end else begin
				begin_read(dma_ext_ram_byte_cpu_addr(src_addr_i[12:0]));
			end
		end
	endtask

	task begin_dma_next_source_read;
		begin
			begin_dma_source_read(dma_source_next_byte_addr(dma_src_addr_q, dma_ctl_q[3]));
		end
	endtask

	task schedule_dma_dest_read;
		reg [15:0] dst_addr_v;
		reg [2:0] dst_kind_v;
		begin
			dst_addr_v = dma_dest_cpu_addr(dma_mode_q, dma_dmvp_q[1], dma_dst_addr_q);
			dst_kind_v = (dma_mode_q == 2'b11) ? AK_EXT : AK_VRAM;
			begin_read_classified(dst_addr_v, dst_kind_v);
			state_q <= ST_DMA_DEST_SAMPLE;
		end
	endtask

	task prepare_dma_packet;
		input [7:0] current_byte_i;
		input [7:0] next_byte_i;
		reg [2:0] packet_pixels_v;
		reg [7:0] packet_byte_v;
		reg full_byte_v;
		begin
			packet_pixels_v = dma_packet_pixel_count(dma_dst_x_q[1:0], dma_line_count_q);
			packet_byte_v = dma_source_packet(current_byte_i, next_byte_i, dma_src_phase_q, dma_ctl_q[3]);
			full_byte_v = (dma_dst_x_q[1:0] == 2'b00) && (packet_pixels_v == 3'd4);
			dma_packet_pixels_q <= packet_pixels_v;
			dma_packet_byte_q <= packet_byte_v;
			if (full_byte_v && dma_ctl_q[0]) begin
				mem_byte_q <= dma_map_byte(dma_dmpl_q, packet_byte_v);
				state_q <= ST_DMA_WRITE_SETUP;
			end else begin
				state_q <= ST_DMA_DEST_SETUP;
			end
		end
	endtask

	wire dma_direct_prefetch_eligible_w =
		((dma_mode_q == 2'b01) || (dma_mode_q == 2'b10)) &&
		dma_ctl_q[0] &&
		(dma_dst_x_q[1:0] == 2'b00) &&
		(dma_packet_pixels_q == 3'd4) &&
		!dma_packet_needs_next_source(dma_src_phase_q, 3'd4, dma_ctl_q[3]) &&
		dma_packet_advances_source_byte(dma_src_phase_q, 3'd4, dma_ctl_q[3]) &&
		(dma_line_count_q >= 8'd7) &&
		(lcc_q[3:1] == 3'b000);

	task advance_dma_after_prefetched_write;
		input [7:0] prefetched_byte_i;
		reg [1:0] next_src_phase_v;
		reg [7:0] next_packet_v;
		begin
			next_src_phase_v = dma_advance_source_phase_packet(dma_src_phase_q, 3'd4, dma_ctl_q[3]);
			next_packet_v = dma_source_packet(prefetched_byte_i, 8'h00, next_src_phase_v, dma_ctl_q[3]);
			dma_line_count_q <= dma_line_count_q - 8'd4;
			dma_src_addr_q <= dma_source_next_byte_addr(dma_src_addr_q, dma_ctl_q[3]);
			dma_src_byte_q <= prefetched_byte_i;
			dma_src_byte_valid_q <= 1'b1;
			dma_src_next_valid_q <= 1'b0;
			dma_dst_addr_q <= dma_dest_next_byte_addr(dma_dst_addr_q);
			dma_src_phase_q <= next_src_phase_v;
			dma_src_x_q <= dma_advance_source_x_packet(dma_src_x_q, 3'd4, dma_ctl_q[3]);
			dma_dst_x_q <= dma_dst_x_q + 8'd4;
			dma_packet_pixels_q <= 3'd4;
			dma_packet_byte_q <= next_packet_v;
			mem_byte_q <= dma_map_byte(dma_dmpl_q, next_packet_v);
			state_q <= ST_DMA_WRITE_SETUP;
		end
	endtask

	task advance_dma_after_write;
		input [2:0] step_pixels_i;
		reg [13:0] next_src_line_v;
		reg [13:0] next_dst_line_v;
		reg [7:0] next_src_y_v;
		reg [7:0] step_minus_one_v;
		reg source_advance_v;
		reg destination_advance_v;
		begin
			next_src_line_v = dma_source_next_line_addr(dma_src_line_q, dma_mode_q, dma_ctl_q[4]);
			next_dst_line_v = dma_dest_next_line_addr(dma_dst_line_q, dma_mode_q);
			next_src_y_v = dma_next_source_y(dma_src_y_q, dma_ctl_q[4]);
			step_minus_one_v = {5'b00000, step_pixels_i} - 8'd1;
			source_advance_v = dma_packet_advances_source_byte(dma_src_phase_q, step_pixels_i, dma_ctl_q[3]);
			destination_advance_v =
				({1'b0, dma_dst_x_q[1:0]} + step_pixels_i) >= 3'd4;
			if (dma_line_count_q == step_minus_one_v) begin
				if (dma_row_count_q == 8'h00) begin
					dma_active_q <= 1'b0;
					dmc_q[7] <= 1'b0;
					ir0_q[7] <= 1'b1;
					irq_dma_pending_q <= 1'b1;
					dma_src_byte_valid_q <= 1'b0;
					dma_src_next_valid_q <= 1'b0;
					state_q <= ST_FETCH_SETUP;
				end else begin
					dma_row_count_q <= dma_row_count_q - 8'h01;
					dma_line_count_q <= dma_arm_line_count_q;
					dma_src_line_q <= next_src_line_v;
					dma_src_addr_q <= next_src_line_v;
					dma_src_phase_q <= dma_arm_src_x_q[1:0];
					dma_src_x_q <= dma_arm_src_x_q;
					dma_dst_line_q <= next_dst_line_v;
					dma_dst_addr_q <= next_dst_line_v;
					dma_dst_x_q <= dma_arm_dst_x_q;
					dma_src_y_q <= next_src_y_v;
					dma_dst_y_q <= dma_dst_y_q + 8'h01;
					dma_src_byte_valid_q <= 1'b0;
					dma_src_next_valid_q <= 1'b0;
					schedule_next_dma_step();
				end
			end else begin
				dma_line_count_q <= dma_line_count_q - {5'b00000, step_pixels_i};
				if (source_advance_v) begin
					dma_src_addr_q <= dma_source_next_byte_addr(dma_src_addr_q, dma_ctl_q[3]);
					dma_src_byte_q <= dma_src_next_byte_q;
					dma_src_byte_valid_q <= dma_src_next_valid_q;
					dma_src_next_valid_q <= 1'b0;
				end
				if (destination_advance_v) begin
					dma_dst_addr_q <= dma_dest_next_byte_addr(dma_dst_addr_q);
				end
				dma_src_phase_q <= dma_advance_source_phase_packet(dma_src_phase_q, step_pixels_i, dma_ctl_q[3]);
				dma_src_x_q <= dma_advance_source_x_packet(dma_src_x_q, step_pixels_i, dma_ctl_q[3]);
				dma_dst_x_q <= dma_dst_x_q + {5'b00000, step_pixels_i};
				schedule_next_dma_step();
			end
		end
	endtask

	//========================================================================
	// Reset state
	//========================================================================
	task apply_core_reset_state;
		begin
			idle_bus();
			pc_q <= 16'h1020;
			sp_q <= 16'h0000;
			ps0_q <= 8'h00;
			ps1_q <= 8'h00;
			ie0_q <= 8'h00;
			ie1_q <= 8'h00;
			ir0_q <= 8'h00;
			ir1_q <= 8'h00;
			irq_dma_pending_q <= 1'b0;
			irq_tim0_pending_q <= 1'b0;
			irq_uart_pending_q <= 1'b0;
			irq_lcdc_pending_q <= 1'b0;
			irq_tim1_pending_q <= 1'b0;
			irq_clk_pending_q <= 1'b0;
			irq_pio_pending_q <= 1'b0;
			sys_q <= 8'h00;
			ckc_q <= 8'h00;
			cpu_clock_select_q <= CKC_RESET_FCPUS;
			cpu_prescale_q <= cpu_prescale_reload(CKC_RESET_FCPUS);
			cpu_phi1_pending_q <= 1'b0;
			cpu_subclock_accum_q <= 25'd0;
			cpu_subclock_phase_q <= 1'b0;
			p0_q <= 8'h00;
			p1_q <= 8'h00;
			p2_q <= 8'h00;
			p3_q <= 8'h00;
			pio_scan_update_q <= 1'b0;
			p0c_q <= P0C_RESET;
			p1c_q <= P1C_RESET;
			p2c_q <= P2C_RESET;
			p3c_q <= P3C_RESET;
			mmu0_q <= 8'h00;
			mmu1_q <= 8'h00;
			mmu2_q <= 8'h00;
			mmu3_q <= 8'h00;
			mmu4_q <= 8'h00;
			mmu1_resolved_q <= 8'h00;
			mmu2_resolved_q <= 8'h00;
			mmu3_resolved_q <= 8'h00;
			mmu4_resolved_q <= 8'h00;
			lcc_q <= 8'hB0;
			lch_q <= 8'h07;
			lcv_q <= 8'h27;
			dmc_q <= 8'h00;
			dmx1_q <= 8'h00;
			dmy1_q <= 8'h00;
			dmdx_q <= 8'h00;
			dmdy_q <= 8'h00;
			dmx2_q <= 8'h00;
			dmy2_q <= 8'h00;
			dmpl_q <= 8'h00;
			dmbr_q <= 8'h00;
			dmvp_q <= 8'h00;
			urtt_q <= 8'hFF;
			urtr_q <= 8'h00;
			urts_q <= 8'h02;
			urtc_q <= 8'h00;
			sgc_q <= 8'h00;
			sg0l_q <= 8'h00;
			sg1l_q <= 8'h00;
			sg2l_q <= 8'h00;
			sg0th_q <= 8'h00;
			sg1th_q <= 8'h00;
			sg2th_q <= 8'h00;
			sg0t_q <= 16'h0000;
			sg1t_q <= 16'h0000;
			sg2t_q <= 16'h0000;
			sgda_q <= 8'h00;
			sgda_write_strobe_q <= 1'b0;
			sound_phi0_div_q <= 1'b0;
			tm0c_q <= 8'h00;
			tm0_reload_q <= 8'h00;
			tm1c_q <= 8'h00;
			tm1_reload_q <= 8'h00;
			tm0c_write_seq_q <= 2'b00;
			tm0_reload_write_seq_q <= 2'b00;
			tm1c_write_seq_q <= 2'b00;
			tm1_reload_write_seq_q <= 2'b00;
			clkt_write_seq_q <= 2'b00;
			clkt_run_q <= 1'b0;
			clkt_minute_q <= 1'b0;
			rtc_toggle_prev_q <= 1'b0;
			rtc_seen_toggle_q <= 1'b0;
			rtc_host_init_done_q <= 1'b0;
			rtc_year_q <= 8'h00;
			rtc_month_q <= 8'h00;
			rtc_day_q <= 8'h00;
			rtc_hour_q <= 8'h00;
			rtc_minute_q <= 8'h00;
			rtc_second_q <= 8'h00;
			uart_tx_active_q <= 1'b0;
			uart_tx_div_q <= 16'h0000;
			uart_tx_shift_q <= 12'hFFF;
			uart_tx_bits_q <= 4'd0;
			uart_rx_active_q <= 1'b0;
			uart_rx_div_q <= 16'h0000;
			reset_uart_rx(1'b0);
			rxdb_prev_q <= 1'b1;
			txdb_q <= 1'b1;
			wdt_q <= 8'h00;
			wdtc_q <= 8'h38;
			wdt_div_q <= wdt_tick_period(3'b000) - 24'd1;
			lcdc_dma_div_q <= lcdc_dma_tick_period(3'b000) - 24'd1;
			lcdc_scan_div_q <= lcdc_dma_tick_period(3'b000) - 24'd1;
			lcdc_shift_q <= 6'd0;
			lcdc_hphase_q <= 5'd0;
			lcdc_line_q <= 8'd0;
			lcdc_vblank_q <= 1'b0;
			lcdc_vblank_prev_q <= 1'b0;
			video_vblank_prev_q <= 1'b0;
			lcdc_fr_q <= 1'b0;
			lcdc_lp_q <= 1'b0;
			lcdc_xc_q <= 1'b0;
			lcdc_scan_byte_q <= 8'h00;
			lcdc_xd_q <= 4'h0;
			lcdc_yd_q <= 1'b0;
			dma_active_q <= 1'b0;
			dma_mode_q <= 2'b00;
			dma_arm_src_x_q <= 8'h00;
			dma_arm_dst_x_q <= 8'h00;
			dma_arm_line_count_q <= 8'h00;
			dma_ctl_q <= 8'h00;
			dma_dmpl_q <= 8'h00;
			dma_dmbr_q <= 8'h00;
			dma_dmvp_q <= 8'h00;
			dma_hdot_200_q <= 1'b0;
			dma_src_byte_q <= 8'h00;
			dma_src_next_byte_q <= 8'h00;
			dma_src_byte_valid_q <= 1'b0;
			dma_src_next_valid_q <= 1'b0;
			dma_packet_pixels_q <= 3'd1;
			dma_packet_byte_q <= 8'h00;
			dma_src_addr_q <= 14'h0000;
			dma_src_line_q <= 14'h0000;
			dma_src_phase_q <= 2'b00;
			dma_src_x_q <= 8'h00;
			dma_src_y_q <= 8'h00;
			dma_dst_addr_q <= 14'h0000;
			dma_dst_line_q <= 14'h0000;
			dma_dst_x_q <= 8'h00;
			dma_dst_y_q <= 8'h00;
			dma_line_count_q <= 8'h00;
			dma_row_count_q <= 8'h00;
			opcode_q <= 8'hFF;
			operand0_q <= 8'h00;
			operand1_q <= 8'h00;
			mem_byte_q <= 8'h00;
			alu_res_q <= 8'h00;
			target_addr_q <= 8'h00;
			target_reg_q <= 4'h0;
			pair_base_q <= 4'h0;
			op_class_q <= CLASS_NONE;
			op_stage_q <= 3'd0;
			state_q <= ST_RESET;
			return_state_q <= ST_FETCH_SETUP;
			stall_count_q <= 6'd0;
			warmup_q <= RESET_WARMUP_CYCLES;
			halted_q <= 1'b0;
			stopped_q <= 1'b0;
			mmu0_en_q <= 1'b0;
			nmi_pending_q <= 1'b0;
			wdt_pending_q <= 1'b0;
			ext_pending_q <= 1'b0;
			ill_pending_q <= 1'b0;
			wdt_reset_pending_q <= 1'b0;
			nmib_prev_q <= nmib_i;
			intb_prev_q <= intb_i;
			power_halt_wake_seen_q <= 1'b0;
			iret_q <= 1'b0;
			irq_resume_defer_q <= 1'b0;
			vector_addr_q <= 16'h0000;
			access_kind_q <= AK_NONE;
			access_addr_q <= 16'h0000;
			access_wdata_q <= 8'h00;
			read_wait_q <= 2'd0;
			warm_boot_sysflag_turnon_suppress_q <= warm_boot_i;
			eff_addr_q <= 16'h0000;
			md_acc_q <= 16'h0000;
			md_shift_q <= 16'h0000;
			md_work_q <= 8'h00;
			md_rem_q <= 9'h000;
			md_count_q <= 5'd0;
			ram_addr_q <= 10'h000;
			ram_wdata_q <= 8'h00;
			rom_addr_q <= 12'h000;
			clk_q <= 1'b0;
			reset_sfr_holes();
`ifndef SYNTHESIS
			init_sfr_shadow_reset();
`endif
		end
	endtask

	cache_ram #(
		.ADDR_WIDTH(10),
		.DATA_WIDTH(8)
	) u_iram (
		.clk_i(clk_sys_i),
		.addr_i(ram_addr_mux_w),
		.wren_i(ram_wren_mux_w),
		.wdata_i(ram_wdata_mux_w),
		.q_o(ram_rdata_w)
	);

	//========================================================================
	// Submodules
	//========================================================================
	sm8521_gp_store u_gp_store (
		.clk_sys_i(clk_sys_i),
		.resetb_i(resetb_i),
		.cold_reset_i(!warm_boot_i),
		.active_bank_i(ps0_q[7:3]),
		.write_mask_a_i(gp_write_mask_a_q),
		.write_data_a_i(gp_write_data_a_q),
		.write_mask_b_i(gp_write_mask_b_q),
		.write_data_b_i(gp_write_data_b_q),
		.init_active_o(gp_store_init_active_q),
		.shadow_o(gp_shadow_flat_w),
		.ss_active_i(ss_gp_sel_w),
		.ss_addr_i(ss_gp_addr_w),
		.ss_wren_i(savestate_wr_i && ss_gp_sel_w),
		.ss_wdata_i(savestate_wdata_i),
		.ss_rdata_o(gp_ss_rdata_w)
	);

	sm8521_sound u_sound (
		.clk_sys_i(clk_sys_i),
		.resetb_i(resetb_i),
		.core_reset_i(sound_core_reset_w),
		.fck_ce_i(phi0_ce_i),
		.stopped_i(stopped_q),
		.sgc_i(sgc_q),
		.sg0l_i(sg0l_q),
		.sg1l_i(sg1l_q),
		.sg2l_i(sg2l_q),
		.sg0t_i(sg0t_q),
		.sg1t_i(sg1t_q),
		.sg2t_i(sg2t_q),
		.sgda_i(sgda_q),
		.sgda_write_strobe_i(sgda_write_strobe_q),
		.sg0_wave_i(sg0_wave_w),
		.sg1_wave_i(sg1_wave_w),
		.sound_pcm_o(sound_pcm_w),
		.ss_addr_i(ss_sound_addr_w),
		.ss_wren_i(savestate_wr_i && ss_sound_sel_w),
		.ss_wdata_i(savestate_wdata_i),
		.ss_rdata_o(sound_ss_rdata_w)
	);

	sm8521_timers #(
		.CLKT_1S_PHI0_CYCLES(CLKT_1S_PHI0_CYCLES)
	) u_timers (
		.clk_sys_i(clk_sys_i),
		.resetb_i(resetb_i),
		.core_reset_i(timers_core_reset_w),
		.phi0_ce_i(phi0_ce_i),
		.main_stopped_i(timers_stopped_w),
		.tm0c_i(tm0c_q),
		.tm0_reload_i(tm0_reload_q),
		.tm1c_i(tm1c_q),
		.tm1_reload_i(tm1_reload_q),
		.tm0c_write_seq_i(tm0c_write_seq_q),
		.tm0_reload_write_seq_i(tm0_reload_write_seq_q),
		.tm1c_write_seq_i(tm1c_write_seq_q),
		.tm1_reload_write_seq_i(tm1_reload_write_seq_q),
		.clkt_write_seq_i(clkt_write_seq_q),
		.clkt_run_i(clkt_run_q),
		.clkt_minute_i(clkt_minute_q),
		.tm0d_q(tm0d_q),
		.tm0_div_q(tm0_div_q),
		.tm0_out_q(tm0_out_q),
		.tm1d_q(tm1d_q),
		.tm1_div_q(tm1_div_q),
		.tm1_out_q(tm1_out_q),
		.clkt_count_q(clkt_count_q),
		.clkt_div_q(clkt_div_q),
		.tm0_irq_event_o(tm0_irq_event_w),
		.tm1_irq_event_o(tm1_irq_event_w),
		.clkt_second_event_o(clkt_second_event_w),
		.clk_irq_event_o(clk_irq_event_w),
		.ss_addr_i(ss_timers_addr_w),
		.ss_wren_i(savestate_wr_i && ss_timers_sel_w),
		.ss_wdata_i(savestate_wdata_i),
		.ss_rdata_o(timers_ss_rdata_w)
	);

	sm8521_boot_rom u_boot_rom (
		.ce_i(1'b1),
		.addr_i(rom_addr_q),
		.data_o(rom_rdata_w)
	);

	sm8521_decode u_decode (
		.opcode_i(opcode_q),
		.action_o(decode_action_w),
		.op_class_o(decode_op_class_w),
		.op_stage_o(decode_op_stage_w),
		.next_state_o(decode_next_state_w),
		.target_reg_we_o(decode_target_reg_we_w),
		.target_reg_o(decode_target_reg_w),
		.pair_base_we_o(decode_pair_base_we_w),
		.pair_base_o(decode_pair_base_w),
		.target_addr_we_o(decode_target_addr_we_w),
		.target_addr_o(decode_target_addr_w)
	);

`ifndef SYNTHESIS
	integer i;
	initial begin
		for (i = 0; i < 264; i = i + 1) begin
			reg_bank_q[i] = 8'h00;
		end
		for (i = 0; i < 256; i = i + 1) begin
			lowmem_q[i] = 8'h00;
		end
		for (i = 0; i < 16; i = i + 1) begin
			sg0w_q[i] = 8'h00;
			sg1w_q[i] = 8'h00;
		end
	end

	//========================================================================
	// Simulation-only inspection helpers
	//========================================================================
	function [7:0] debug_reg_bank_read;
		input [8:0] idx;
		begin
			debug_reg_bank_read = u_gp_store.debug_bank_read(idx);
		end
	endfunction

	function [7:0] debug_gp_read;
		input [3:0] idx;
		begin
			debug_gp_read = gp_read(idx);
		end
	endfunction

	function [7:0] debug_direct_read;
		input [7:0] addr;
		begin
			debug_direct_read = direct_read(addr);
		end
	endfunction

	function [7:0] debug_lowmem_read;
		input [7:0] addr;
		begin
			debug_lowmem_read = direct_read(addr);
		end
	endfunction

	function [7:0] debug_iram_read;
		input [9:0] addr;
		begin
			debug_iram_read = u_iram.mem_q[addr];
		end
	endfunction

	task debug_iram_write;
		input [9:0] addr;
		input [7:0] data;
		begin
			u_iram.mem_q[addr] = data;
			if (addr[9:7] == 3'b001) begin
				iram_lo_shadow_q[addr[6:0]] = data;
				lowmem_q[addr[7:0]] = data;
			end
		end
	endtask

	task debug_reg_bank_write;
		input [8:0] idx;
		input [7:0] data;
		begin
			u_gp_store.debug_bank_write(idx, data);
			reg_bank_q[idx] = data;
		end
	endtask

	task debug_lowmem_write;
		input [7:0] addr;
		input [7:0] data;
		begin
			lowmem_q[addr] = data;
			if (addr < 8'h10) begin
				debug_reg_bank_write(reg_bank_index(addr[3:0]), data);
			end else if (addr[7]) begin
				debug_iram_write({2'b00, addr}, data);
			end else begin
				sfr_shadow_q[addr[6:0]] = data;
				debug_write_sfr_hole(addr, data);
			end
		end
	endtask
`endif

	always @(*) begin
		gp_shadow_q[0] = gp_shadow_flat_w[7:0];
		gp_shadow_q[1] = gp_shadow_flat_w[15:8];
		gp_shadow_q[2] = gp_shadow_flat_w[23:16];
		gp_shadow_q[3] = gp_shadow_flat_w[31:24];
		gp_shadow_q[4] = gp_shadow_flat_w[39:32];
		gp_shadow_q[5] = gp_shadow_flat_w[47:40];
		gp_shadow_q[6] = gp_shadow_flat_w[55:48];
		gp_shadow_q[7] = gp_shadow_flat_w[63:56];
		gp_shadow_q[8] = gp_shadow_flat_w[71:64];
		gp_shadow_q[9] = gp_shadow_flat_w[79:72];
		gp_shadow_q[10] = gp_shadow_flat_w[87:80];
		gp_shadow_q[11] = gp_shadow_flat_w[95:88];
		gp_shadow_q[12] = gp_shadow_flat_w[103:96];
		gp_shadow_q[13] = gp_shadow_flat_w[111:104];
		gp_shadow_q[14] = gp_shadow_flat_w[119:112];
		gp_shadow_q[15] = gp_shadow_flat_w[127:120];
	end

	//========================================================================
	// The CPU
	//========================================================================
	always @(posedge clk_sys_i) begin
		// Start of beat.
		//
		// The state machine never performs an access itself: it records what it
		// wants and the drains at the bottom of this block carry it out. That is
		// what keeps each decode in one place instead of inlined at every one of
		// its call sites. Clear last beat's requests first.
		bus_wr_req_v = 1'b0;
		bus_wr_addr_v = 16'h0000;
		bus_wr_data_v = 8'h00;
		bus_rd_req_v = 1'b0;
		bus_rd_addr_v = 16'h0000;
		bus_rd_kind_v = AK_NONE;
		bus_rd_kind_known_v = 1'b0;
		bus_rd_phys_v = 1'b0;
		bus_rd_phys_addr_v = 21'h000000;
		bus_rd_pick_state_v = 1'b0;
		bus_rd_wait_state_v = ST_RESET;
		bus_rd_sample_state_v = ST_RESET;

		agen_kind_v = AGEN_NONE;
		agen_desc_v = 8'h00;
		agen_imm_v = 16'h0000;

		dwr_count_v = 2'd0;
		dwr_addr0_v = 8'h00;
		dwr_data0_v = 8'h00;
		dwr_addr1_v = 8'h00;
		dwr_data1_v = 8'h00;
		dwr_addr2_v = 8'h00;
		dwr_data2_v = 8'h00;

		// Register-indirect addressing, shared by the classes that use it.
		// operand0_q[7:6] is the mode and operand0_q[2:0] the pointer register:
		//
		//   00  @Rn        01  @Rn+ (post-increment)
		//   10  @Rn + d8   11  @-Rn (pre-decrement)
		//
		// The indexed form runs a second stage, after the displacement byte has
		// been fetched, pointing with target_addr_q[2:0] instead.
		rmb_ptr_v   = gp_read({1'b0, operand0_q[2:0]});
		rmb_index_v = gp_read({1'b0, target_addr_q[2:0]});
		rmb_addr_v  = (operand0_q[7:6] == 2'b11) ? rmb_ptr_v - 8'h01 : rmb_ptr_v;

		// The address rd_ind_v reads this beat.
		case (op_class_q)
			// PUSH @Rr and the indirect unary group point with operand0_q[5:3].
			CLASS_INDIRECT_REG_OP: indirect_addr_v = gp_read({1'b0, operand0_q[5:3]});
			// The bit-modify form indexes that same pointer with operand1_q.
			CLASS_RI_BIT_MODIFY: indirect_addr_v = operand1_q + gp_read({1'b0, operand0_q[5:3]});
			// The displacement is the freshly fetched byte here, and the byte
			// stashed in mem_byte_q in the two classes below.
			CLASS_BYTE_INDIRECT_OP: indirect_addr_v = (op_stage_q == 3'b000) ? rmb_addr_v :
				(operand0_q + ((target_addr_q[2:0] != 3'b000) ? rmb_index_v : 8'h00));
			CLASS_INDIRECT_CMP, CLASS_INDIRECT_MOV:
				indirect_addr_v = (op_stage_q == 3'b000) ? rmb_addr_v :
					(mem_byte_q + rmb_index_v);
			default: indirect_addr_v = rmb_addr_v;
		endcase

		// MOVM and word-immediate operations consume target_addr_q, but never
		// the operand1 read ports. Share that pair before the direct/SFR decode;
		// the other classes retain operand1_q and all read side effects remain
		// at their original execute sites. Five physical read ports cover the
		// seven logical values without changing the beat or adding registers.
		case (op_class_q)
			CLASS_MOVM_MASK, CLASS_WORD_IMM_OP: rd_op1_addr_v = target_addr_q;
			default: rd_op1_addr_v = operand1_q;
		endcase
		rd_op0_v    = direct_read(operand0_q);
		rd_op0_hi_v = direct_read(operand0_q + 8'h01);
		rd_op1_v    = direct_read(rd_op1_addr_v);
		rd_op1_hi_v = direct_read(rd_op1_addr_v + 8'h01);
		rd_tgt_v    = rd_op1_v;
		rd_tgt_hi_v = rd_op1_hi_v;
		rd_ind_v    = direct_read(indirect_addr_v);

		// The LCDC keeps fetching its scanline through idle bus beats, and
		// idle_bus is inlined at every idle state, so its address is resolved
		// here rather than in all forty-seven copies.
		lcdc_scan_active_v = lcc_q[7] && !dma_active_q && !lcdc_vblank_q &&
			({1'b0, lcdc_shift_q} < vram_line_bytes_sel(lch_q[5]));
		lcdc_scan_addr_v = vram_dma_addr({2'b00, lcdc_shift_q}, lcdc_line_q);
		ram_wren_q <= 1'b0;
		gp_write_mask_a_q <= 8'h00;
		gp_write_mask_b_q <= 8'h00;
		sgda_write_strobe_q <= 1'b0;
		pio_scan_update_q <= 1'b0;

		if (!savestate_pause_req_i) begin
			savestate_pause_ready_q <= 1'b0;
			savestate_clock_frozen_q <= 1'b0;
		end else if (savestate_pause_ready_q || savestate_active_i) begin
			savestate_clock_frozen_q <= 1'b1;
		end

		if (savestate_active_i) begin
			// ---- Savestate writes and warm-boot seeding ----
			if (savestate_wr_i && ss_scalar_sel_w) begin
				savestate_scalar_write(savestate_addr_i[7:0], savestate_wdata_i);
			end
			if (savestate_wr_i && ss_iram_sel_w && (ss_iram_addr_w[9:7] == 3'b001)) begin
				iram_lo_shadow_q[ss_iram_addr_w[6:0]] <= savestate_wdata_i;
			end
		end else if (!resetb_i) begin
			savestate_pause_ready_q <= 1'b0;
			gp_write_data_a_q <= 64'h0000000000000000;
			gp_write_data_b_q <= 64'h0000000000000000;
`ifndef SYNTHESIS
			if (!warm_boot_i) begin
				for (reg_reset_i = 0; reg_reset_i < 264; reg_reset_i = reg_reset_i + 1) begin
					reg_bank_q[reg_reset_i] <= 8'h00;
				end
			end
			for (sfr_shadow_reset_i = 0; sfr_shadow_reset_i < 128; sfr_shadow_reset_i = sfr_shadow_reset_i + 1) begin
				sfr_shadow_q[sfr_shadow_reset_i] <= 8'hFF;
			end
`endif
			if (!warm_boot_i) begin
				for (iram_lo_shadow_reset_i = 0; iram_lo_shadow_reset_i < 128; iram_lo_shadow_reset_i = iram_lo_shadow_reset_i + 1) begin
					iram_lo_shadow_q[iram_lo_shadow_reset_i] <= 8'h00;
				end
				// SDK sources document SYSFLAG bit 0 as "1 = initialize from power on".
				// Only seed that cold-boot hint on an actual power-style reset.
				iram_lo_shadow_q[7'h3C] <= 8'h01;
			end
`ifndef SYNTHESIS
			if (!warm_boot_i) begin
				for (lowmem_reset_i = 0; lowmem_reset_i < 256; lowmem_reset_i = lowmem_reset_i + 1) begin
					lowmem_q[lowmem_reset_i] <= 8'hFF;
				end
				lowmem_q[8'hBC] <= 8'h01;
			end
`endif
			if (!warm_boot_i) begin
				for (sg_reset_i = 0; sg_reset_i < 16; sg_reset_i = sg_reset_i + 1) begin
					sg0w_q[sg_reset_i] <= 8'h00;
					sg1w_q[sg_reset_i] <= 8'h00;
				end
			end
			apply_core_reset_state();
		end else begin
			if (!savestate_clock_frozen_q) begin
				if (cpu_subclock_tick_w) begin
					cpu_subclock_accum_q <= cpu_subclock_accum_q - SUBCLOCK_RELOAD_THRESHOLD;
					cpu_subclock_phase_q <= ~cpu_subclock_phase_q;
				end else begin
					cpu_subclock_accum_q <= cpu_subclock_accum_q + SUBCLOCK_HZ;
				end
			end

			// ---- CPU clock: the CKC prescaler divides phi0 down to the core ----
			if (phi0_ce_i && (cpu_clock_select_q <= 3'b100)) begin
				if (cpu_prescale_q != 4'd0) begin
					cpu_prescale_q <= cpu_prescale_q - 4'd1;
				end else begin
					cpu_prescale_q <= cpu_prescale_reload(cpu_clock_select_q);
				end
			end

			if (cpu_core_phi0_ce_w) begin
				cpu_phi1_pending_q <= 1'b1;
			end
			if (cpu_core_phi1_ce_w) begin
				cpu_phi1_pending_q <= 1'b0;
			end

			// STOP wake stabilization is counted in main-clock periods, not in
			// CKC-selected CPU clocks. The existing half-count uses fCK/2 phi0.
			if (phi0_ce_i && (state_q == ST_RESET) && (warmup_q != 18'd0)) begin
				warmup_q <= warmup_q - 18'd1;
			end

			// ---- Peripherals: watchdog, interrupt inputs, LCDC scanout ----
			if (phi0_ce_i) begin
				if (wdt_reset_pending_q) begin
					// Treat watchdog overflow reset as a CPU core reset, not a low-RAM or wave-RAM scrub.
					apply_core_reset_state();
				end else begin
					sound_phi0_div_q <= ~sound_phi0_div_q;
					lcdc_lp_q <= 1'b0;
					lcdc_xc_q <= 1'b0;
					lcdc_yd_q <= 1'b0;

					// STOP halts fc-derived watchdog clocks; fx selectors use the
					// still-running sub-clock and continue to count.
					if (wdtc_q[7] && (!stopped_q || wdtc_q[2])) begin
						if (wdt_div_q != 24'd0) begin
							wdt_div_q <= wdt_div_q - 24'd1;
						end else begin
							wdt_div_q <= wdt_tick_period(wdtc_q[2:0]) - 24'd1;
							wdt_q <= wdt_q + 8'h01;
							if (wdt_q == 8'hFF) begin
								if (wdtc_q[6]) begin
									wdt_pending_q <= 1'b1;
								end else begin
									wdt_reset_pending_q <= 1'b1;
								end
							end
						end
					end

					// MAME blanks display-off scanlines without advancing its
					// Game.com LCD scanline. Leaving these registers untouched
					// freezes the emulated LCDC while the video adapter blanks output.
					if (!stopped_q && lcc_q[7]) begin
						if (!dma_active_q && !lcdc_vblank_q &&
							({1'b0, lcdc_shift_q} < vram_line_bytes_sel(lch_q[5]))) begin
							lcdc_scan_byte_q <= vd_din_i;
							lcdc_xd_q <= lcdc_scan_xd(lcdc_scan_byte_q, lcdc_fr_q);
						end else begin
							lcdc_scan_byte_q <= 8'h00;
							lcdc_xd_q <= 4'h0;
						end
						if (lcdc_scan_div_q != 24'd0) begin
							lcdc_scan_div_q <= lcdc_scan_div_q - 24'd1;
						end else begin
							lcdc_scan_div_q <= lcdc_dma_tick_period(lcc_q[3:1]) - 24'd1;

							if (!lcdc_vblank_q && (lcdc_shift_q < lcdc_shift_clocks(lch_q[5]))) begin
								lcdc_xc_q <= 1'b1;
							end

							if ((lcdc_shift_q + 6'd1) >= lcdc_shift_clocks(lch_q[5])) begin
								lcdc_shift_q <= 6'd0;
								if (!lcdc_vblank_q) begin
									lcdc_lp_q <= 1'b1;
								end

								if (lcdc_hphase_q >= lch_q[4:0]) begin
									lcdc_hphase_q <= 5'd0;
									lcdc_yd_q <= 1'b1;

									if ((lcdc_line_q + 8'd1) >= (lcdc_active_lines(lcv_q[5:4]) + {4'h0, lcv_q[3:0]})) begin
										lcdc_line_q <= 8'd0;
										lcdc_vblank_q <= 1'b0;
										lcdc_fr_q <= ~lcdc_fr_q;
									end else begin
										lcdc_line_q <= lcdc_line_q + 8'd1;
										lcdc_vblank_q <= ((lcdc_line_q + 8'd1) >= lcdc_active_lines(lcv_q[5:4]));
									end
								end else begin
									lcdc_hphase_q <= lcdc_hphase_q + 5'd1;
								end
							end else begin
								lcdc_shift_q <= lcdc_shift_q + 6'd1;
							end
						end
					end

					if (!stopped_q && lcc_q[7] && !lcdc_vblank_prev_q && lcdc_vblank_q && (lcv_q[3:0] != 4'h0)) begin
						// Raise LCDCINT on the internal LCDC VBlank edge. The
						// MiSTer video adapter has extra host blanking for centering,
						// but software-visible LCDC timing should follow LCV.
						ir0_q[0] <= 1'b1;
						irq_lcdc_pending_q <= 1'b1;
					end

					if (nmib_prev_q && !nmib_i) nmi_pending_q <= 1'b1;
					if (intb_prev_q && !intb_i) begin
						ext_pending_q <= 1'b1;
						ir0_q[4] <= 1'b1;
					end
					// GUESS: power_stop_wake_i is a board/MiSTer integration
					// wake source, not a documented SM8521 INTB edge. Do not
					// synthesize EXTINT/IR0[4] from it unless hardware evidence
					// shows the Game.com power path really reaches the CPU INTB pin.
					// MAME's Game.com board model does not synthesize a generic
					// PIO-change interrupt either; it only updates the port bytes when
					// software writes the mux selects. Keep the PIO vector available,
					// but do not invent an always-on edge source here. GUESS: real PIO
					// interrupt edge/select behavior still needs hardware evidence.
					nmib_prev_q <= nmib_i;
					intb_prev_q <= intb_i;
					if (!stopped_q) begin
						lcdc_vblank_prev_q <= lcdc_vblank_q;
						video_vblank_prev_q <= vr_i;
					end
					if (!power_stop_wake_i) begin
						power_halt_wake_seen_q <= 1'b0;
					end
				end
			end

			if (cpu_core_phi0_ce_w && !wdt_reset_pending_q) begin
				clk_q <= 1'b1;
			end

			// ---- The phi0 half: bus transactions and the state machine ----
			// CPU bus/state work follows the CKC-selected positive phase, which
			// the CKC prescaler can make slower than peripheral phi0. Once DMA
			// owns the buses it uses only the unprescaled peripheral cadence
			// until it releases the halted CPU.
			if (!wdt_reset_pending_q &&
				((cpu_core_phi0_ce_w && !dma_active_q) ||
				(phi0_ce_i && dma_active_q))) begin
				begin : cpu_phi0_state_work
					case (state_q)
						ST_RESET: begin
							idle_bus();
							if (((warmup_q == 18'd0) ||
								(phi0_ce_i && (warmup_q == 18'd1))) &&
								!gp_store_init_active_q) begin
								state_q <= ST_FETCH_SETUP;
							end
						end

						ST_FETCH_SETUP: begin
							begin : st_fetch_setup_logic
								idle_bus();
								if (savestate_pause_req_i && !dma_active_q && (warmup_q == 18'd0) && !gp_store_init_active_q && !wdt_reset_pending_q) begin
									savestate_pause_ready_q <= 1'b1;
								end else if (stopped_q) begin
									if (wdt_pending_q || nmi_pending_q) begin
										halted_q <= 1'b0;
										stopped_q <= 1'b0;
										warmup_q <= stop_warmup_cycles(ckc_q[1:0]);
										state_q <= ST_RESET;
									end else if (power_stop_wake_i) begin
										// GUESS: wake the FPGA integration from STOP
										// without inventing a CPU-visible EXT request.
										halted_q <= 1'b0;
										stopped_q <= 1'b0;
										power_halt_wake_seen_q <= 1'b1;
										warmup_q <= stop_warmup_cycles(ckc_q[1:0]);
										state_q <= ST_RESET;
									end else if (stop_wake_irq_source_w != IRQ_NONE) begin
										halted_q <= 1'b0;
										stopped_q <= 1'b0;
										warmup_q <= stop_warmup_cycles(ckc_q[1:0]);
										state_q <= ST_RESET;
									end
								end else if (wdt_pending_q) begin
									wdt_pending_q <= 1'b0;
									start_interrupt(16'h101C, IRQ_NONE);
								end else if (nmi_pending_q) begin
									nmi_pending_q <= 1'b0;
									start_interrupt(16'h101E, IRQ_NONE);
								end else if (ill_pending_q) begin
									ill_pending_q <= 1'b0;
									start_interrupt(16'h101E, IRQ_NONE);
								end else if (irq_resume_defer_q && !halted_q) begin
									// The SM85CPU instruction timing diagram overlaps opcode
									// prefetch with execution. Model IRET as allowing the
									// restored instruction to reach the next fetch boundary before
									// maskable IRQ arbitration can redirect again, while retaining
									// any private request that arrived during the handler.
									irq_resume_defer_q <= 1'b0;
									schedule_read(pc_q, ST_FETCH_WAIT, ST_FETCH_SAMPLE);
								end else if (takeable_irq_source_w != IRQ_NONE) begin
									// MAME's SM8500 core processes pending CPU interrupt inputs
									// before calling its HALT-time DMA callback. Keep that
									// ordering for accepted interrupts; stale visible request bits
									// do not participate in takeability.
									start_interrupt(irq_vector_addr(takeable_irq_source_w), takeable_irq_source_w);
								end else if (halted_q && power_stop_wake_i && !power_halt_wake_seen_q && ie0_q[4] && !dmc_q[7]) begin
									// GUESS: Game.com routes the front-panel Power key as an
									// external-request wake for HALT, but only synthesize it
									// while HALTed so ordinary runtime key holds do not leave
									// stale EXTINT state behind.
									ext_pending_q <= 1'b1;
									ir0_q[4] <= 1'b1;
									power_halt_wake_seen_q <= 1'b1;
									halted_q <= 1'b0;
									schedule_read(pc_q, ST_FETCH_WAIT, ST_FETCH_SAMPLE);
								end else if (halted_q && (halt_wake_irq_source_w != IRQ_NONE) && !dmc_q[7]) begin
									// HALT is shallow standby: the interrupt source wakes the CPU,
									// while IE/priority/global-I decide whether a vector is taken.
									// Leaving standby is not acceptance, so the request stands.
									halted_q <= 1'b0;
									schedule_read(pc_q, ST_FETCH_WAIT, ST_FETCH_SAMPLE);
								end else if ((pending_irq_source_w != IRQ_NONE) && !(halted_q && dmc_q[7])) begin
									// The datasheet starts interrupt processing once all conditions
									// are set up, so a request the hardware latched survives being
									// observed while IE, global-I or priority block it. It is
									// consumed by interrupt entry or by software clearing IR0/IR1,
									// never by the CPU merely noticing it.
									if (!halted_q) begin
										schedule_read(pc_q, ST_FETCH_WAIT, ST_FETCH_SAMPLE);
									end
								end else if (dmc_q[7] && halted_q) begin
									if (!dma_active_q) begin
										dma_active_q <= 1'b1;
										// GUESS: DMC[7] arms DMA, but the register file is sampled
										// when HALT actually dispatches the blitter. The datasheet
										// only says "set the start bit and execute HALT"; this
										// MAME-compatible snapshot point keeps DM* writes immediately
										// before the DMA-starting HALT effective.
										dma_arm_src_x_q <= dmx1_q;
										dma_arm_dst_x_q <= dmx2_q;
										dma_arm_line_count_q <= dmdx_q;
										dma_mode_q <= dmc_q[2:1];
										dma_ctl_q <= dmc_q;
										dma_dmpl_q <= dmpl_q;
										dma_dmbr_q <= dmbr_q;
										dma_dmvp_q <= dmvp_q;
										// GUESS: keep VRAM row geometry fixed for the active blit
										// instead of reading live LCDC state mid-transfer. The
										// datasheet defines LCH[5]'s row size but not writes racing
										// an already-running DMA.
										dma_hdot_200_q <= lch_q[5];
										dma_src_byte_q <= 8'h00;
										dma_src_next_byte_q <= 8'h00;
										dma_src_byte_valid_q <= 1'b0;
										dma_src_next_valid_q <= 1'b0;
										dma_packet_pixels_q <= 3'd1;
										dma_packet_byte_q <= 8'h00;
										dma_src_addr_q <= dma_source_byte_addr(dmc_q[2:1], dmx1_q, dmy1_q, lch_q[5]);
										dma_src_line_q <= dma_source_byte_addr(dmc_q[2:1], dmx1_q, dmy1_q, lch_q[5]);
										dma_src_phase_q <= dmx1_q[1:0];
										dma_src_x_q <= dmx1_q;
										dma_src_y_q <= dmy1_q;
										dma_dst_addr_q <= dma_dest_byte_addr(dmc_q[2:1], dmx2_q, dmy2_q, lch_q[5]);
										dma_dst_line_q <= dma_dest_byte_addr(dmc_q[2:1], dmx2_q, dmy2_q, lch_q[5]);
										dma_dst_x_q <= dmx2_q;
										dma_dst_y_q <= dmy2_q;
										dma_line_count_q <= dmdx_q;
										dma_row_count_q <= dmdy_q;
										schedule_next_dma_step();
									end else begin
										schedule_next_dma_step();
									end
								end else if (!halted_q) begin
									schedule_read(pc_q, ST_FETCH_WAIT, ST_FETCH_SAMPLE);
								end
							end
						end

						ST_FETCH_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_FETCH_SAMPLE;
							end
						end

						ST_FETCH_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								opcode_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								pc_q <= pc_q + 16'h0001;
								idle_bus();
								state_q <= ST_DECODE;
							end
						end

						ST_OP_READ8_SETUP: begin
							idle_bus();
							schedule_read(pc_q, ST_OP_READ8_WAIT, ST_OP_READ8_SAMPLE);
						end

						ST_OP_READ8_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_OP_READ8_SAMPLE;
							end
						end

						ST_OP_READ8_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								take_operand_byte(ST_EXECUTE);
							end
						end

						ST_OP_READ8_16H_SETUP: begin
							idle_bus();
							schedule_read(pc_q, ST_OP_READ8_16H_WAIT, ST_OP_READ8_16H_SAMPLE);
						end

						ST_OP_READ8_16H_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_OP_READ8_16H_SAMPLE;
							end
						end

						ST_OP_READ8_16H_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								take_operand_byte(ST_OP_READ8_16L_SETUP);
							end
						end

						ST_OP_READ8_16L_SETUP: begin
							idle_bus();
							schedule_read(pc_q, ST_OP_READ8_16L_WAIT, ST_OP_READ8_16L_SAMPLE);
						end

						ST_OP_READ8_16L_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_OP_READ8_16L_SAMPLE;
							end
						end

						ST_OP_READ8_16L_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								operand1_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								pc_q <= pc_q + 16'h0001;
								idle_bus();
								state_q <= ST_EXECUTE;
							end
						end

						ST_OP_READ16H_SETUP: begin
							idle_bus();
							schedule_read(pc_q, ST_OP_READ16H_WAIT, ST_OP_READ16H_SAMPLE);
						end

						ST_OP_READ16H_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_OP_READ16H_SAMPLE;
							end
						end

						ST_OP_READ16H_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								take_operand_byte(ST_OP_READ16L_SETUP);
							end
						end

						ST_OP_READ16L_SETUP: begin
							idle_bus();
							schedule_read(pc_q, ST_OP_READ16L_WAIT, ST_OP_READ16L_SAMPLE);
						end

						ST_OP_READ16L_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_OP_READ16L_SAMPLE;
							end
						end

						ST_OP_READ16L_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								operand1_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								pc_q <= pc_q + 16'h0001;
								idle_bus();
								state_q <= ST_EXECUTE;
							end
						end

						ST_STALL: begin
							idle_bus();
							if (stall_count_q != 6'd0) begin
								stall_count_q <= stall_count_q - 6'd1;
							end else begin
								state_q <= return_state_q;
							end
						end

						ST_MEM_READ_SETUP: begin
							idle_bus();
							schedule_read(eff_addr_q, ST_MEM_READ_WAIT, ST_MEM_READ_SAMPLE);
						end

						ST_MEM_READ_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_MEM_READ_SAMPLE;
							end
						end

						ST_MEM_READ_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								mem_byte_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								idle_bus();
								state_q <= ST_EXECUTE;
							end
						end

						ST_MEM_WRITE_SETUP: begin
							idle_bus();
							begin_write(eff_addr_q, mem_byte_q);
							state_q <= ST_MEM_WRITE_SAMPLE;
						end

						ST_MEM_WRITE_SAMPLE: begin
							idle_bus();
							case (access_kind_q)
								AK_GP, AK_SFR: direct_write(access_addr_q[7:0], access_wdata_q);
								AK_IRAM: begin
									// begin_write already loaded the RAM port, so
									// only the strobe is left to raise here.
									ram_wren_q <= 1'b1;
									if (rtc_iram_addr(access_addr_q)) begin
										write_rtc_mirror(access_addr_q, access_wdata_q);
									end
									// Only page zero is shadowed.
									if (access_addr_q[15:8] == 8'h00) begin
										write_iram_shadow(access_addr_q[7:0], access_wdata_q);
									end
								end
								default: begin
								end
							endcase
							state_q <= return_state_q;
						end

						ST_DMA_PACE: begin
							idle_bus();
							if (lcdc_dma_div_q != 24'd0) begin
								lcdc_dma_div_q <= lcdc_dma_div_q - 24'd1;
							end else begin
								state_q <= ST_DMA_READ_SETUP;
							end
						end

						ST_MULDIV_STEP: begin
							idle_bus();
							if (op_class_q == CLASS_MUL_RR || op_class_q == CLASS_MUL_IMM) begin
								if (md_count_q < 5'd8) begin
									if (md_work_q[0]) begin
										md_acc_q <= md_acc_q + md_shift_q;
									end
									md_shift_q <= {md_shift_q[14:0], 1'b0};
									md_work_q <= {1'b0, md_work_q[7:1]};
									md_count_q <= md_count_q + 5'd1;
								end else begin
									direct_write_word(target_addr_q, md_acc_q);
									ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V)) | ((md_acc_q == 16'h0000) ? FLAG_Z : 8'h00);
									stall_count_q <= 6'd8;
									return_state_q <= ST_FETCH_SETUP;
									state_q <= ST_STALL;
								end
							end else begin
								if (md_count_q < 5'd16) begin
									if ({md_rem_q[7:0], md_shift_q[15]} >= {1'b0, md_work_q}) begin
										md_rem_q <= {md_rem_q[7:0], md_shift_q[15]} - {1'b0, md_work_q};
										md_acc_q <= {md_acc_q[14:0], 1'b1};
									end else begin
										md_rem_q <= {md_rem_q[7:0], md_shift_q[15]};
										md_acc_q <= {md_acc_q[14:0], 1'b0};
									end
									md_shift_q <= {md_shift_q[14:0], 1'b0};
									md_count_q <= md_count_q + 5'd1;
								end else begin
									// The external BIOS formatter consumes the quotient from the
									// destination and the remainder from source.high. Quiz Wiz also
									// corroborates Z as a test of the completed quotient.
									direct_write_word(target_addr_q, md_acc_q);
									if (op_class_q == CLASS_DIV_RR) begin
										direct_write(operand0_q, md_rem_q[7:0]);
										ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V)) | ((md_acc_q == 16'h0000) ? FLAG_Z : 8'h00);
										stall_count_q <= 6'd23;
									end else begin
										ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V)) | ((md_acc_q == 16'h0000) ? FLAG_Z : 8'h00);
										stall_count_q <= 6'd20;
									end
									return_state_q <= ST_FETCH_SETUP;
									state_q <= ST_STALL;
								end
							end
						end

						ST_DMA_READ_SETUP: begin
							begin : st_dma_read_setup_logic
								reg [2:0] packet_pixels_v;
								idle_bus();
								packet_pixels_v = dma_packet_pixel_count(dma_dst_x_q[1:0], dma_line_count_q);
								if (dma_src_byte_valid_q) begin
									if (dma_packet_needs_next_source(dma_src_phase_q, packet_pixels_v, dma_ctl_q[3])) begin
										begin_dma_next_source_read();
										state_q <= ST_DMA_WINDOW_SAMPLE;
									end else begin
										// The remaining source pixels are already protected by the
										// byte latch. Preserve the two-cycle source phase without
										// rereading same-page VRAM after a destination write.
										state_q <= ST_DMA_SOURCE_HOLD;
									end
								end else begin
									begin_dma_source_read(dma_src_addr_q);
									state_q <= ST_DMA_READ_SAMPLE;
								end
							end
						end

						ST_DMA_READ_SAMPLE: begin
							if (!dma_wait_for_rom_w) begin : st_dma_read_sample_logic
								reg [7:0] src_byte_v;
								reg [2:0] packet_pixels_v;
								src_byte_v = access_rdata_cheat_w;
								packet_pixels_v = dma_packet_pixel_count(dma_dst_x_q[1:0], dma_line_count_q);
								dma_src_byte_q <= src_byte_v;
								dma_src_byte_valid_q <= 1'b1;
								if (dma_packet_needs_next_source(dma_src_phase_q, packet_pixels_v, dma_ctl_q[3])) begin
									// Release the completed request before giving the neighboring
									// byte its own full setup/sample transaction.
									idle_bus();
									state_q <= ST_DMA_WINDOW_SETUP;
								end else begin
									// End the sampled request now so a following dual-bus pair
									// begins with a distinct external read transaction.
									idle_bus();
									prepare_dma_packet(src_byte_v, dma_src_next_byte_q);
								end
							end
						end

						ST_DMA_WINDOW_SETUP: begin
							idle_bus();
							begin_dma_next_source_read();
							state_q <= ST_DMA_WINDOW_SAMPLE;
						end

						ST_DMA_WINDOW_SAMPLE: begin
							if (!dma_wait_for_rom_w) begin : st_dma_window_sample_logic
								reg [7:0] next_byte_v;
								next_byte_v = access_rdata_cheat_w;
								dma_src_next_byte_q <= next_byte_v;
								dma_src_next_valid_q <= 1'b1;
								prepare_dma_packet(dma_src_byte_q, next_byte_v);
							end
						end

						ST_DMA_SOURCE_HOLD: begin
							idle_bus();
							prepare_dma_packet(dma_src_byte_q, dma_src_next_byte_q);
						end

						ST_DMA_DEST_SETUP: begin
							schedule_dma_dest_read();
						end

						ST_DMA_DEST_SAMPLE: begin
							begin : st_dma_dest_sample_logic
								reg [7:0] dst_byte_v;
								reg [7:0] out_byte_v;
								dst_byte_v = access_rdata_cheat_w;
								if ((dma_dst_x_q[1:0] == 2'b00) && (dma_packet_pixels_q == 3'd4)) begin
									if (dma_ctl_q[0]) begin
										out_byte_v = dma_map_byte(dma_dmpl_q, dma_packet_byte_q);
									end else begin
										out_byte_v = dma_compound_merge(dma_dmpl_q, dma_packet_byte_q, dst_byte_v);
									end
								end else begin
									out_byte_v = dma_packet_merge(
										dma_dmpl_q,
										dma_packet_byte_q,
										dst_byte_v,
										dma_dst_x_q[1:0],
										dma_packet_pixels_q,
										dma_ctl_q[0]
									);
								end
								mem_byte_q <= out_byte_v;
								state_q <= ST_DMA_WRITE_SETUP;
							end
						end

						ST_DMA_WRITE_SETUP: begin
							idle_bus();
							begin_write(dma_dest_cpu_addr(dma_mode_q, dma_dmvp_q[1], dma_dst_addr_q), mem_byte_q);
							if (dma_direct_prefetch_eligible_w) begin
								// The external and VRAM buses start together and remain asserted
								// through the following common sample/completion cycle.
								begin_dma_next_source_read();
							end
							state_q <= ST_DMA_WRITE_SAMPLE;
						end

						ST_DMA_WRITE_SAMPLE: begin
							// A delayed ROM prefetch may extend this state. The VRAM
							// strobe then repeats the same address/data write; no DMA
							// counters or packet latches advance until ROM is ready.
							if (!dma_wait_for_rom_w) begin
								if ((access_kind_q == AK_EXT) &&
									((dma_mode_q == 2'b01) || (dma_mode_q == 2'b10))) begin
									begin : st_dma_write_sample_prefetch_logic
										reg [7:0] prefetched_byte_v;
										prefetched_byte_v = access_rdata_cheat_w;
										idle_bus();
										advance_dma_after_prefetched_write(prefetched_byte_v);
									end
								end else begin
									idle_bus();
									advance_dma_after_write(dma_packet_pixels_q);
								end
							end
						end

						ST_INT_PUSH0_SETUP: begin
							push_byte_now(pc_q[7:0]);
							state_q <= ST_INT_PUSH0_SAMPLE;
						end

						ST_INT_PUSH0_SAMPLE: begin
							idle_bus();
							if (access_kind_q == AK_IRAM) ram_wren_q <= 1'b1;
							state_q <= ST_INT_PUSH1_SETUP;
						end

						ST_INT_PUSH1_SETUP: begin
							push_byte_now(pc_q[15:8]);
							state_q <= ST_INT_PUSH1_SAMPLE;
						end

						ST_INT_PUSH1_SAMPLE: begin
							idle_bus();
							if (access_kind_q == AK_IRAM) ram_wren_q <= 1'b1;
							state_q <= ST_INT_PUSH2_SETUP;
						end

						ST_INT_PUSH2_SETUP: begin
							push_byte_now(ps1_q);
							state_q <= ST_INT_PUSH2_SAMPLE;
						end

						ST_INT_PUSH2_SAMPLE: begin
							idle_bus();
							if (access_kind_q == AK_IRAM) ram_wren_q <= 1'b1;
							ps1_q[0] <= 1'b0;
							state_q <= ST_INT_VECH_SETUP;
						end

						ST_INT_VECH_SETUP: begin
							idle_bus();
							schedule_read(vector_addr_q, ST_INT_VECH_WAIT, ST_INT_VECH_SAMPLE);
						end

						ST_INT_VECH_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_INT_VECH_SAMPLE;
							end
						end

						ST_INT_VECH_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								operand0_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								idle_bus();
								state_q <= ST_INT_VECL_SETUP;
							end
						end

						ST_INT_VECL_SETUP: begin
							idle_bus();
							schedule_read(vector_addr_q + 16'h0001, ST_INT_VECL_WAIT, ST_INT_VECL_SAMPLE);
						end

						ST_INT_VECL_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_INT_VECL_SAMPLE;
							end
						end

						ST_INT_VECL_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								pc_q <= {operand0_q, access_rdata_cheat_w};
								idle_bus();
								state_q <= ST_FETCH_SETUP;
							end
						end

						ST_RET_POP0_SETUP: begin
							idle_bus();
							schedule_read(sp_q, ST_RET_POP0_WAIT, ST_RET_POP0_SAMPLE);
						end

						ST_RET_POP0_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_RET_POP0_SAMPLE;
							end
						end

						ST_RET_POP0_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								operand0_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								sp_q <= stack_inc(sp_q);
								idle_bus();
								if (iret_q) begin
									state_q <= ST_RET_POP1_SETUP;
								end else begin
									state_q <= ST_RET_POP2_SETUP;
								end
							end
						end

						ST_RET_POP1_SETUP: begin
							idle_bus();
							schedule_read(sp_q, ST_RET_POP1_WAIT, ST_RET_POP1_SAMPLE);
						end

						ST_RET_POP1_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_RET_POP1_SAMPLE;
							end
						end

						ST_RET_POP1_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								operand1_q <= access_rdata_cheat_w;
								apply_read_side_effects(access_kind_q, access_addr_q);
								sp_q <= stack_inc(sp_q);
								idle_bus();
								ps1_q <= operand0_q;
								state_q <= ST_RET_POP2_SETUP;
							end
						end

						ST_RET_POP2_SETUP: begin
							idle_bus();
							schedule_read(sp_q, ST_RET_POP2_WAIT, ST_RET_POP2_SAMPLE);
						end

						ST_RET_POP2_WAIT: begin
							if (read_wait_q != 2'd0) begin
								read_wait_q <= read_wait_q - 2'd1;
							end else begin
								state_q <= ST_RET_POP2_SAMPLE;
							end
						end

						ST_RET_POP2_SAMPLE: begin
							if (!sample_wait_for_rom(access_kind_q, access_addr_q, rom_read_ready_i)) begin
								idle_bus();
								sp_q <= stack_inc(sp_q);
								// RET is 10/8 cycles and IRET 12/10, selected by the
								// 16-bit stack bit SYS[6]. Both carry the same two-cycle
								// wide-stack premium.
								if (iret_q) begin
									pc_q <= {operand1_q, access_rdata_cheat_w};
									stall_count_q <= sys_q[6] ? 6'd3 : 6'd1;
									if (ps1_q[0]) begin
										irq_resume_defer_q <= 1'b1;
									end
								end else begin
									pc_q <= {operand0_q, access_rdata_cheat_w};
									stall_count_q <= sys_q[6] ? 6'd3 : 6'd1;
								end
								iret_q <= 1'b0;
								return_state_q <= ST_FETCH_SETUP;
								state_q <= ST_STALL;
							end
						end

						default: begin
						end
					endcase
				end
			end

			// ---- Peripherals: real-time clock and UART ----
			if (phi0_ce_i) begin
				if (!wdt_reset_pending_q) begin

					if (rtc_i[64] != rtc_toggle_prev_q) begin
						rtc_toggle_prev_q <= rtc_i[64];
						rtc_seen_toggle_q <= 1'b1;
					end

					if (clkt_run_q && !rtc_host_init_done_q &&
						(rtc_seen_toggle_q || (rtc_i[64] != rtc_toggle_prev_q))) begin
						rtc_host_init_done_q <= 1'b1;
						rtc_year_q <= rtc_i[47:40];
						rtc_month_q <= rtc_i[39:32];
						rtc_day_q <= rtc_i[31:24];
						rtc_hour_q <= rtc_i[23:16];
						rtc_minute_q <= rtc_i[15:8];
						rtc_second_q <= rtc_i[7:0];
					end else if (clkt_second_event_w) begin
						if (rtc_second_q == 8'h59) begin
							rtc_second_q <= 8'h00;
							if (rtc_minute_q == 8'h59) begin
								rtc_minute_q <= 8'h00;
								if (rtc_hour_q == 8'h23) begin
									rtc_hour_q <= 8'h00;
									if (rtc_day_q == rtc_days_in_month(rtc_month_q, rtc_year_q)) begin
										rtc_day_q <= 8'h01;
										if (rtc_month_q == 8'h12) begin
											rtc_month_q <= 8'h01;
											if (rtc_year_q == 8'h99) begin
												rtc_year_q <= 8'h00;
											end else begin
												rtc_year_q <= rtc_bcd_increment(rtc_year_q);
											end
										end else begin
											rtc_month_q <= rtc_bcd_increment(rtc_month_q);
										end
									end else begin
										rtc_day_q <= rtc_bcd_increment(rtc_day_q);
									end
								end else begin
									rtc_hour_q <= rtc_bcd_increment(rtc_hour_q);
								end
							end else begin
								rtc_minute_q <= rtc_bcd_increment(rtc_minute_q);
							end
						end else begin
							rtc_second_q <= rtc_bcd_increment(rtc_second_q);
						end
					end

					// Hardware exposes IR0/IR1 as sticky request latches, so timer compare
					// events set the visible bits. Private request latches separate CPU
					// delivery from software-visible status.
					if (tm0_irq_event_w) begin
						ir0_q[6] <= 1'b1;
						irq_tim0_pending_q <= 1'b1;
					end

					if (tm1_irq_event_w) begin
						ir1_q[6] <= 1'b1;
						irq_tim1_pending_q <= 1'b1;
					end

					if (clk_irq_event_w) begin
						ir1_q[4] <= 1'b1;
						irq_clk_pending_q <= 1'b1;
					end

					if (!stopped_q) begin
						if (urtc_q[3] && !uart_rx_active_q && rxdb_prev_q && !rxdb_i) begin
							uart_rx_active_q <= 1'b1;
							uart_rx_div_q <= uart_half_bit_period_w - 16'd1;
							reset_uart_rx(1'b1);
							urts_q[5] <= 1'b1;
						end else if (uart_rx_active_q && tm0c_q[7]) begin
							if (uart_rx_div_q != 16'h0000) begin
								uart_rx_div_q <= uart_rx_div_q - 16'h0001;
							end else begin
								case (uart_rx_state_q)
									3'd0: begin
										if (!rxdb_i) begin
											uart_rx_state_q <= 3'd1;
											uart_rx_div_q <= uart_bit_period_w - 16'd1;
										end else begin
											uart_rx_active_q <= 1'b0;
											urts_q[5] <= 1'b0;
										end
									end

									3'd1: begin
										uart_rx_shift_q[uart_rx_bit_q[2:0]] <= rxdb_i;
										uart_rx_parity_q <= uart_rx_parity_q ^ rxdb_i;
										if (uart_rx_bit_q == 4'd7) begin
											if (uart_rx_paren_q) begin
												uart_rx_state_q <= 3'd2;
											end else begin
												uart_rx_state_q <= 3'd3;
											end
										end
										uart_rx_bit_q <= uart_rx_bit_q + 4'd1;
										uart_rx_div_q <= uart_bit_period_w - 16'd1;
									end

									3'd2: begin
										if (rxdb_i != (uart_rx_odd_q ? ~uart_rx_parity_q : uart_rx_parity_q)) begin
											uart_rx_pe_q <= 1'b1;
										end
										uart_rx_state_q <= 3'd3;
										uart_rx_div_q <= uart_bit_period_w - 16'd1;
									end

									3'd3: begin
										if (!rxdb_i) uart_rx_fe_q <= 1'b1;
										if (uart_rx_stop2_q) begin
											uart_rx_state_q <= 3'd4;
											uart_rx_div_q <= uart_bit_period_w - 16'd1;
										end else begin
											uart_rx_active_q <= 1'b0;
											urts_q[5] <= 1'b0;
											if (urts_q[0]) begin
												urts_q[4] <= 1'b1;
											end else begin
												urtr_q <= uart_rx_shift_q;
												urts_q[0] <= 1'b1;
											end
											if (uart_rx_pe_q) urts_q[2] <= 1'b1;
											if (uart_rx_fe_q || !rxdb_i) urts_q[3] <= 1'b1;
											ir0_q[3] <= 1'b1;
											irq_uart_pending_q <= 1'b1;
										end
									end

									default: begin
										if (!rxdb_i) uart_rx_fe_q <= 1'b1;
										uart_rx_active_q <= 1'b0;
										urts_q[5] <= 1'b0;
										if (urts_q[0]) begin
											urts_q[4] <= 1'b1;
										end else begin
											urtr_q <= uart_rx_shift_q;
											urts_q[0] <= 1'b1;
										end
										if (uart_rx_pe_q) urts_q[2] <= 1'b1;
										if (uart_rx_fe_q || !rxdb_i) urts_q[3] <= 1'b1;
										ir0_q[3] <= 1'b1;
										irq_uart_pending_q <= 1'b1;
									end
								endcase
							end
						end

						if (uart_tx_active_q && tm0c_q[7]) begin
							if (uart_tx_div_q != 16'h0000) begin
								uart_tx_div_q <= uart_tx_div_q - 16'h0001;
							end else if (uart_tx_bits_q != 4'd0) begin
								txdb_q <= uart_tx_shift_q[0];
								uart_tx_shift_q <= {1'b1, uart_tx_shift_q[11:1]};
								uart_tx_bits_q <= uart_tx_bits_q - 4'd1;
								uart_tx_div_q <= uart_bit_period_w - 16'd1;
							end else begin
								txdb_q <= 1'b1;
								uart_tx_active_q <= 1'b0;
								urts_q[1] <= 1'b1;
								ir0_q[3] <= 1'b1;
								irq_uart_pending_q <= 1'b1;
							end
						end
					end

					rxdb_prev_q <= rxdb_i;
				end
			end

			// ---- The phi1 half: decode and execute ----
			if (cpu_core_phi1_ce_w) begin
				clk_q <= 1'b0;

				case (state_q)
					ST_DECODE: begin
						case (decode_action_w)
							ACT_NORMAL: begin
								op_class_q <= decode_op_class_w;
								op_stage_q <= decode_op_stage_w;
								if (decode_target_reg_we_w) target_reg_q <= decode_target_reg_w;
								if (decode_pair_base_we_w) pair_base_q <= decode_pair_base_w;
								if (decode_target_addr_we_w) target_addr_q <= decode_target_addr_w;
								state_q <= decode_next_state_w;
							end

							ACT_FETCH: begin
								state_q <= ST_FETCH_SETUP;
							end

							ACT_ILL: begin
								ill_pending_q <= 1'b1;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_HALT: begin
								halted_q <= 1'b1;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_STOP: begin
								halted_q <= 1'b0;
								// The datasheet makes CKC[5:3] effective only when
								// FCPUEN is set and STOP executes. Reserved selections
								// retain the current clock. Stop-Free Boot suppresses
								// sleep below, but the instruction still commits CKC.
								if (ckc_q[7] &&
									((ckc_q[5:3] <= 3'b100) || (ckc_q[5:3] == 3'b111))) begin
									cpu_clock_select_q <= ckc_q[5:3];
									cpu_prescale_q <= cpu_prescale_reload(ckc_q[5:3]);
								end
								// Allow MiSTer-side stop-free booting without inventing a new
								// opcode path: when requested, treat STOP as a plain flow-
								// through instruction instead of entering the STOP state.
								stopped_q <= stop_disable_i ? 1'b0 : 1'b1;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_RET: begin
								iret_q <= 1'b0;
								state_q <= ST_RET_POP0_SETUP;
							end

							ACT_IRET: begin
								iret_q <= 1'b1;
								state_q <= ST_RET_POP0_SETUP;
							end

							ACT_CLRC: begin
								ps1_q <= ps1_q & ~FLAG_C;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_COMC: begin
								ps1_q <= ps1_q ^ FLAG_C;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_SETC: begin
								ps1_q <= ps1_q | FLAG_C;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_EI: begin
								ps1_q <= ps1_q | FLAG_I;
								state_q <= ST_FETCH_SETUP;
							end

							ACT_DI: begin
								ps1_q <= ps1_q & ~FLAG_I;
								state_q <= ST_FETCH_SETUP;
							end

							default: begin
								ill_pending_q <= 1'b1;
								state_q <= ST_FETCH_SETUP;
							end
						endcase
					end

					ST_EXECUTE: begin
						case (op_class_q)
							CLASS_CLR_FIXED: begin
								direct_write(operand0_q, 8'h00);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOVPS0_IMM: begin
								ps0_q <= operand0_q;
`ifndef SYNTHESIS
								refresh_gp_mirror(operand0_q[7:3]);
`endif
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOVI_REG: begin
								gp_write(target_reg_q, operand0_q);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOVI_SFR: begin
								direct_write(target_addr_q, operand0_q);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOVI_FIXED: begin
								direct_write(operand1_q, operand0_q);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_DIRECT_UNARY: begin
								exec_direct_unary_op(opcode_q, operand0_q, {rd_op0_v, rd_op0_hi_v});
								op_stage_q <= 3'd0;
								case (opcode_q)
									8'h01: enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
									8'h0D: enter_core_stall(core_fetch_resume_stall(4'd3), ST_FETCH_SETUP);
									8'h18, 8'h19:
										enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
									default: state_q <= ST_FETCH_SETUP;
								endcase
							end

							CLASS_COMPACT_BYTE_OP: begin
								if (operand0_q[7:6] == 2'b00) begin
									exec_mem_byte_alu(opcode_q, {1'b0, operand0_q[5:3]}, gp_read({1'b0, operand0_q[2:0]}));
									enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
								end else begin
									ill_pending_q <= 1'b1;
									state_q <= ST_FETCH_SETUP;
								end
								op_stage_q <= 3'd0;
							end

							CLASS_STACK_DIRECT: begin
								// Ratio-normalized stack timing: one-byte operations are
								// 8/10 cycles and two-byte operations are 10/12 cycles.
								if (opcode_q == 8'h0E) begin
									if (op_stage_q == 3'd0) begin
										push_byte(rd_op0_v);
										apply_direct_read_side_effects(operand0_q);
										op_stage_q <= 3'd1;
										return_state_q <= ST_EXECUTE;
										state_q <= ST_MEM_WRITE_SETUP;
									end else begin
										op_stage_q <= 3'd0;
										if (sys_q[6]) begin
											enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
										end
									end
								end else if (opcode_q == 8'h0F) begin
									if (op_stage_q == 3'd0) begin
										eff_addr_q <= sp_q;
										op_stage_q <= 3'd1;
										state_q <= ST_MEM_READ_SETUP;
									end else begin
										direct_write(operand0_q, mem_byte_q);
										sp_q <= stack_inc(sp_q);
										op_stage_q <= 3'd0;
										if (sys_q[6]) begin
											enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
										end
									end
								end else if (opcode_q == 8'h1E) begin
									if (op_stage_q == 3'd0) begin
										push_byte(rd_op0_hi_v);
										apply_direct_read_side_effects(operand0_q + 8'h01);
										op_stage_q <= 3'd1;
										return_state_q <= ST_EXECUTE;
										state_q <= ST_MEM_WRITE_SETUP;
									end else if (op_stage_q == 3'd1) begin
										push_byte(rd_op0_v);
										apply_direct_read_side_effects(operand0_q);
										op_stage_q <= 3'd2;
										return_state_q <= ST_EXECUTE;
										state_q <= ST_MEM_WRITE_SETUP;
									end else begin
										op_stage_q <= 3'd0;
										if (sys_q[6]) begin
											enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
										end
									end
								end else begin
									if (op_stage_q == 3'd0) begin
										eff_addr_q <= sp_q;
										op_stage_q <= 3'd1;
										state_q <= ST_MEM_READ_SETUP;
									end else if (op_stage_q == 3'd1) begin
										direct_write(operand0_q, mem_byte_q);
										sp_q <= stack_inc(sp_q);
										eff_addr_q <= stack_inc(sp_q);
										op_stage_q <= 3'd2;
										state_q <= ST_MEM_READ_SETUP;
									end else begin
										direct_write(operand0_q + 8'h01, mem_byte_q);
										sp_q <= stack_inc(sp_q);
										op_stage_q <= 3'd0;
										if (sys_q[6]) begin
											enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
										end
									end
								end
							end

							CLASS_INDIRECT_REG_OP: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									if ((opcode_q == 8'h1B) && (operand0_q[2:0] == 3'b110)) begin : class_indirect_reg_push_start
										reg [15:0] next_sp_v;
										next_sp_v = stack_dec(sp_q);
										sp_q <= next_sp_v;
										eff_addr_q <= next_sp_v;
										// PUSH @Rr pushes the byte the register points at,
										// like every other sub-op in this opcode.
										mem_byte_q <= rd_ind_v;
										op_stage_q <= 3'b011;
										return_state_q <= ST_EXECUTE;
										state_q <= ST_MEM_WRITE_SETUP;
									end else if ((opcode_q == 8'h1B) && (operand0_q[2:0] == 3'b111)) begin
										eff_addr_q <= sp_q;
										op_stage_q <= 3'b100;
										state_q <= ST_MEM_READ_SETUP;
									end else begin
										exec_indirect_unary_op(
											opcode_q,
											operand0_q[2:0],
											gp_read({1'b0, operand0_q[5:3]}),
											rd_ind_v
										);
										op_stage_q <= 3'b000;
										enter_core_stall(
											core_fetch_resume_stall(indirect_unary_extra_cycles(opcode_q, operand0_q[2:0])),
											ST_FETCH_SETUP
										);
									end
								end else if (op_stage_q == 3'b011) begin
									op_stage_q <= 3'b000;
									if (sys_q[6]) begin
										enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
									end else begin
										enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
									end
								end else if (op_stage_q == 3'b100) begin
									// POP @Rr stores through the register, not into it.
									direct_write(gp_read({1'b0, target_addr_q[5:3]}), mem_byte_q);
									sp_q <= stack_inc(sp_q);
									op_stage_q <= 3'b000;
									if (sys_q[6]) begin
										enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
									end else begin
										enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
									end
								end else begin
									op_stage_q <= 3'b000;
									state_q <= ST_FETCH_SETUP;
								end
							end

							CLASS_RI_BIT_MODIFY: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									if (operand0_q[5:3] != 3'b000) begin : class_ri_bit_modify_indexed
										reg [7:0] addr_v;
										reg [7:0] value_v;
										addr_v = operand1_q + gp_read({1'b0, operand0_q[5:3]});
										value_v = rd_ind_v;
										apply_direct_read_side_effects(addr_v);
										if (opcode_q == 8'h1C) begin
											direct_write(addr_v, value_v & ~(8'h01 << operand0_q[2:0]));
										end else begin
											direct_write(addr_v, value_v | (8'h01 << operand0_q[2:0]));
										end
										op_stage_q <= 3'b000;
										enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
									end else begin
										request_addr(AGEN_RI, operand0_q, {8'h00, operand1_q});
										op_stage_q <= 3'b001;
										state_q <= ST_MEM_READ_SETUP;
									end
								end else if (op_stage_q == 3'b001) begin
									if (opcode_q == 8'h1C) begin
										mem_byte_q <= mem_byte_q & ~(8'h01 << target_addr_q[2:0]);
									end else begin
										mem_byte_q <= mem_byte_q | (8'h01 << target_addr_q[2:0]);
									end
									op_stage_q <= 3'b010;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else begin
									op_stage_q <= 3'b000;
									enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
								end
							end

							CLASS_RI_BIT_BRANCH: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									alu_res_q <= operand1_q;
									op_stage_q <= 3'b001;
									state_q <= ST_OP_READ8_SETUP;
								end else if (op_stage_q == 3'b001) begin
									request_addr(AGEN_RI, target_addr_q, {8'h00, alu_res_q});
									op_stage_q <= 3'b010;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									if (((opcode_q == 8'h2A) && ((mem_byte_q & (8'h01 << target_addr_q[2:0])) == 8'h00)) ||
										((opcode_q == 8'h2B) && ((mem_byte_q & (8'h01 << target_addr_q[2:0])) != 8'h00))) begin
										pc_q <= pc_q + {{8{operand0_q[7]}}, operand0_q};
										if (target_addr_q[5:3] != 3'b000) begin
											enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd6), ST_FETCH_SETUP);
										end
									end else begin
										if (target_addr_q[5:3] != 3'b000) begin
											state_q <= ST_FETCH_SETUP;
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
										end
									end
									op_stage_q <= 3'b000;
								end
							end

							CLASS_EXTS_DIRECT: begin
								exec_exts_direct_op(operand0_q, rd_op0_hi_v);
								op_stage_q <= 3'd0;
								enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
							end

							CLASS_BTST_DIRECT: begin
								exec_btst_direct_op(operand0_q, operand1_q, rd_op0_v);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_DM: begin
								// Sacred identifies both two-byte encodings, but their
								// operation is unknown. Do not invent a register-file read.
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							// @Rn, @Rn+ and @-Rn resolve in the first stage. The indexed
							// form, mode 10, fetches its displacement byte first and comes
							// back as stage 1 with the descriptor kept in target_addr_q.
							CLASS_BYTE_INDIRECT_OP: begin : class_byte_indirect
								reg [7:0] desc_v;
								if ((op_stage_q == 3'b000) && (operand0_q[7:6] == 2'b10)) begin
									target_addr_q <= operand0_q;
									op_stage_q <= 3'b001;
									state_q <= ST_OP_READ8_SETUP;
								end else begin
									desc_v = (op_stage_q == 3'b000) ? operand0_q : target_addr_q;
									if (op_stage_q == 3'b000) update_rmb_pointer();
									if (opcode_q == 8'h29) begin
										direct_write(indirect_addr_v, source_after_rmb_update(desc_v));
									end else begin
										apply_direct_read_side_effects(indirect_addr_v);
										if (opcode_q <= 8'h27) begin
											exec_mem_byte_alu(opcode_q, {1'b0, desc_v[5:3]}, rd_ind_v);
										end else begin
											gp_write({1'b0, desc_v[5:3]}, rd_ind_v);
										end
									end
									op_stage_q <= 3'b000;
									enter_core_stall(
										core_fetch_resume_stall(rmb_fetch_resume_cycles(opcode_q, desc_v)),
										ST_FETCH_SETUP
									);
								end
							end

							CLASS_DIRECT_IMM_OP: begin
								if (opcode_q == 8'h58) begin
									direct_write(operand1_q, operand0_q);
								end else begin
									exec_direct_imm_op(opcode_q, operand1_q, rd_op1_v, operand0_q);
								end
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MUL_RR: begin : class_mul_rr
								reg [7:0] source_v;
								reg [7:0] target_low_addr_v;
								reg [7:0] target_low_v;
								// Shipped sound-scaling routines corroborate dest.low * src.
								// Odd pairs are unreliable per Sharp; ORing bit 0 is the MAME
								// fallback and fixes Fighters Megamix's MULT 0Dh use.
								target_low_addr_v = operand1_q | 8'h01;
								target_low_v = (operand1_q[0] ? rd_op1_v : rd_op1_hi_v);
								source_v = after_sfr_reads(
									operand0_q,
									rd_op0_v,
									target_low_addr_v == 8'h2C,
									target_low_addr_v == 8'h2D
								);
								target_addr_q <= operand1_q;
								md_acc_q <= 16'h0000;
								md_shift_q <= {8'h00, target_low_v};
								md_work_q <= source_v;
								apply_direct_read_side_effects(target_low_addr_v);
								apply_direct_read_side_effects(operand0_q);
								md_rem_q <= 9'h000;
								md_count_q <= 5'd0;
								op_stage_q <= 3'd0;
								state_q <= ST_MULDIV_STEP;
							end

							CLASS_MUL_IMM: begin : class_mul_imm
								reg [7:0] target_low_addr_v;
								reg [7:0] target_low_v;
								// Game arithmetic also corroborates the immediate form's
								// destination-low input and 16-bit destination result.
								target_low_addr_v = operand1_q | 8'h01;
								target_low_v = (operand1_q[0] ? rd_op1_v : rd_op1_hi_v);
								target_addr_q <= operand1_q;
								md_acc_q <= 16'h0000;
								md_shift_q <= {8'h00, target_low_v};
								apply_direct_read_side_effects(target_low_addr_v);
								md_work_q <= operand0_q;
								md_rem_q <= 9'h000;
								md_count_q <= 5'd0;
								op_stage_q <= 3'd0;
								state_q <= ST_MULDIV_STEP;
							end

							CLASS_DIV_RR: begin : class_div_rr
								reg divisor_uartr_read_v;
								reg divisor_uarts_read_v;
								reg [7:0] divisor_addr_v;
								reg [7:0] divisor_v;
								reg [7:0] dividend_high_v;
								reg [7:0] dividend_low_v;
								// BIOS decimal formatting and game sound scaling corroborate
								// source.low as the divisor. ORing bit 0 for an odd source is
								// only the MAME fallback; no shipped odd DIV use was confirmed.
								divisor_addr_v = operand0_q | 8'h01;
								divisor_uartr_read_v = divisor_addr_v == 8'h2C;
								divisor_uarts_read_v = divisor_addr_v == 8'h2D;
								divisor_v = (operand0_q[0] ? rd_op0_v : rd_op0_hi_v);
								ps1_q <= ps1_q & ~(FLAG_Z | FLAG_V);
								apply_direct_read_side_effects(divisor_addr_v);
								if (divisor_v != 8'h00) begin
									dividend_high_v = after_sfr_reads(
										operand1_q,
										rd_op1_v,
										divisor_uartr_read_v,
										divisor_uarts_read_v
									);
									dividend_low_v = after_sfr_reads(
										operand1_q + 8'h01,
										rd_op1_hi_v,
										divisor_uartr_read_v || (operand1_q == 8'h2C),
										divisor_uarts_read_v || (operand1_q == 8'h2D)
									);
									apply_direct_read_word_side_effects(operand1_q);
									target_addr_q <= operand1_q;
									md_acc_q <= 16'h0000;
									md_shift_q <= {dividend_high_v, dividend_low_v};
									md_work_q <= divisor_v;
									md_rem_q <= 9'h000;
									md_count_q <= 5'd0;
									op_stage_q <= 3'd0;
									state_q <= ST_MULDIV_STEP;
								end else begin
									stall_count_q <= 6'd40;
									return_state_q <= ST_FETCH_SETUP;
									op_stage_q <= 3'd0;
									ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V)) | FLAG_V;
									state_q <= ST_STALL;
								end
							end

							CLASS_DIV_IMM: begin : class_div_imm
								reg [7:0] dividend_high_v;
								reg [7:0] dividend_low_v;
								// Quiz Wiz branches on Z immediately after this form, confirming
								// that Z reports a zero quotient, not a zero input operand.
								ps1_q <= ps1_q & ~(FLAG_Z | FLAG_V);
								if (operand0_q != 8'h00) begin
									dividend_high_v = rd_op1_v;
									dividend_low_v = after_sfr_reads(
										operand1_q + 8'h01,
										rd_op1_hi_v,
										operand1_q == 8'h2C,
										operand1_q == 8'h2D
									);
									apply_direct_read_word_side_effects(operand1_q);
									target_addr_q <= operand1_q;
									md_acc_q <= 16'h0000;
									md_shift_q <= {dividend_high_v, dividend_low_v};
									md_work_q <= operand0_q;
									md_rem_q <= 9'h000;
									md_count_q <= 5'd0;
									op_stage_q <= 3'd0;
									state_q <= ST_MULDIV_STEP;
								end else begin
									stall_count_q <= 6'd37;
									return_state_q <= ST_FETCH_SETUP;
									op_stage_q <= 3'd0;
									ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V)) | FLAG_V;
									state_q <= ST_STALL;
								end
							end

							CLASS_BMOV_BF: begin
								begin : class_bmov_bf
									reg [7:0] target_v;
									reg [7:0] mask_v;
									target_v = rd_op1_v;
									apply_direct_read_side_effects(operand1_q);
									mask_v = (8'h01 << operand0_q[2:0]);
									alu_res_q <= target_v;
									if (operand0_q[6]) begin
										if (ps1_q[1]) begin
											direct_write(operand1_q, target_v | mask_v);
										end else begin
											direct_write(operand1_q, target_v & ~mask_v);
										end
									end else begin
										alu_res_q <= target_v & mask_v;
										ps1_q <= ps1_q & (FLAG_C | FLAG_S | FLAG_D | FLAG_H | FLAG_I);
										if ((target_v & mask_v) != 8'h00) begin
											ps1_q <= (ps1_q & (FLAG_C | FLAG_S | FLAG_D | FLAG_H | FLAG_I)) | FLAG_B;
										end else begin
											ps1_q <= (ps1_q & (FLAG_C | FLAG_S | FLAG_D | FLAG_H | FLAG_I)) | FLAG_Z;
										end
									end
								end
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_BF_LOGIC: begin
								begin : class_bf_logic
									reg [7:0] target_v;
									reg [7:0] mask_v;
									reg [7:0] bit_v;
									reg [7:0] carry_bit_v;
									target_v = rd_op1_v;
									apply_direct_read_side_effects(operand1_q);
									mask_v = (8'h01 << operand0_q[2:0]);
									bit_v = target_v & mask_v;
									carry_bit_v = ps1_q[1] ? mask_v : 8'h00;
									alu_res_q <= bit_v;
									if (operand0_q[7:6] == 2'b00) begin
										// Quiz Wiz and Jeopardy use BR NZ as the mismatch path
										// immediately after BCMP, corroborating Z-on-equality.
										ps1_q <= ps1_q & ~(FLAG_Z | FLAG_V);
										if (bit_v == carry_bit_v) begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V)) | FLAG_Z;
										end
									end else if (operand0_q[7:6] == 2'b01) begin
										ps1_q <= ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B);
										if ((bit_v & carry_bit_v) != 8'h00) begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B)) | FLAG_B;
										end else begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B)) | FLAG_Z;
										end
									end else if (operand0_q[7:6] == 2'b10) begin
										ps1_q <= ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B);
										if ((bit_v | carry_bit_v) != 8'h00) begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B)) | FLAG_B;
										end else begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B)) | FLAG_Z;
										end
									end else begin
										ps1_q <= ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B);
										if ((bit_v ^ carry_bit_v) != 8'h00) begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B)) | FLAG_B;
										end else begin
											ps1_q <= (ps1_q & ~(FLAG_Z | FLAG_V | FLAG_B)) | FLAG_Z;
										end
									end
								end
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							// CMP against an immediate, in the same four addressing forms.
							// The immediate is operand1_q directly, or the byte stashed in
							// mem_byte_q once the indexed form has fetched its displacement.
							CLASS_INDIRECT_CMP: begin
								if ((op_stage_q == 3'b000) && (operand0_q[7:6] == 2'b10)) begin
									target_addr_q <= operand0_q;
									mem_byte_q <= operand1_q;
									op_stage_q <= 3'b001;
									state_q <= ST_OP_READ8_SETUP;
								end else begin
									if (op_stage_q == 3'b000) begin
										target_addr_q <= operand0_q;
										update_rmb_pointer();
									end
									apply_direct_read_side_effects(indirect_addr_v);
									flags_cmp8(rd_ind_v,
										(op_stage_q == 3'b000) ? operand1_q : operand0_q);
									op_stage_q <= 3'b000;
									// Updating the pointer costs one more cycle.
									enter_core_stall(core_fetch_resume_stall(
										((op_stage_q == 3'b000) && (operand0_q[7:6] != 2'b00)) ?
											4'd2 : 4'd1),
										ST_FETCH_SETUP);
								end
							end

							// MOV of an immediate, in the same four addressing forms.
							CLASS_INDIRECT_MOV: begin
								if ((op_stage_q == 3'b000) && (operand0_q[7:6] == 2'b10)) begin
									target_addr_q <= operand0_q;
									mem_byte_q <= operand1_q;
									op_stage_q <= 3'b001;
									state_q <= ST_OP_READ8_SETUP;
								end else if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									update_rmb_pointer();
									direct_write(indirect_addr_v, operand1_q);
									op_stage_q <= 3'b000;
									if (operand0_q[7:6] == 2'b00) begin
										state_q <= ST_FETCH_SETUP;
									end else begin
										// Updating the pointer costs one more cycle.
										enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
									end
								end else begin
									direct_write(indirect_addr_v, operand0_q);
									op_stage_q <= 3'b000;
									state_q <= ST_FETCH_SETUP;
								end
							end

							CLASS_MOVM_MASK: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									mem_byte_q <= operand1_q;
									op_stage_q <= 3'b001;
									state_q <= ST_OP_READ8_SETUP;
								end else begin
									begin : class_movm_mask
										reg [7:0] target_v;
										reg [7:0] source_v;
										target_v = rd_tgt_v;
										apply_direct_read_side_effects(target_addr_q);
										if (opcode_q == 8'h5E) begin
											source_v = rd_op0_v;
											apply_direct_read_side_effects(operand0_q);
											direct_write(target_addr_q, (target_v & mem_byte_q) | source_v);
										end else begin
											direct_write(target_addr_q, (target_v & mem_byte_q) | operand0_q);
										end
									end
									op_stage_q <= 3'b000;
									if (opcode_q == 8'h5E) begin
										enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
									end else begin
										state_q <= ST_FETCH_SETUP;
									end
								end
							end

							CLASS_WORD_RR_OP: begin
								exec_word_rr_op(opcode_q, operand1_q, operand0_q, {rd_op1_v, rd_op1_hi_v}, {rd_op0_v, rd_op0_hi_v});
								op_stage_q <= 3'd0;
								// Real hardware retains four internal cycles for word logic
								// beyond the otherwise identical arithmetic access path.
								case (opcode_q)
									8'h60: enter_core_stall(core_fetch_resume_stall(4'd3), ST_FETCH_SETUP);
									8'h61, 8'h62, 8'h63, 8'h64:
										enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
									default: enter_core_stall(core_fetch_resume_stall(4'd8), ST_FETCH_SETUP);
								endcase
							end

							CLASS_WORD_IMM_OP: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									op_stage_q <= 3'b001;
									state_q <= ST_OP_READ16H_SETUP;
								end else begin
									exec_word_imm_op(opcode_q, target_addr_q, {rd_tgt_v, rd_tgt_hi_v}, {operand0_q, operand1_q});
									op_stage_q <= 3'b000;
									case (opcode_q)
										8'h68: enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
										8'h69, 8'h6A, 8'h6B, 8'h6C:
											enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
										default: enter_core_stall(core_fetch_resume_stall(4'd5), ST_FETCH_SETUP);
									endcase
								end
							end

							CLASS_BIT_BRANCH: begin
								begin : class_bit_branch
									reg [7:0] target_v;
									reg [7:0] mask_v;
									target_v = rd_op0_v;
									apply_direct_read_side_effects(operand0_q);
									mask_v = (8'h01 << opcode_q[2:0]);
									alu_res_q <= target_v;
									if ((opcode_q[3] && ((target_v & mask_v) != 8'h00)) ||
										(!opcode_q[3] && ((target_v & mask_v) == 8'h00))) begin
										pc_q <= pc_q + {{8{operand1_q[7]}}, operand1_q};
										stall_count_q <= 6'd3;
										return_state_q <= ST_FETCH_SETUP;
										state_q <= ST_STALL;
									end else begin
										state_q <= ST_FETCH_SETUP;
									end
								end
								op_stage_q <= 3'd0;
							end

							CLASS_BIT_MODIFY: begin
								begin : class_bit_modify
									reg [7:0] target_v;
									reg [7:0] mask_v;
									target_v = rd_op0_v;
									apply_direct_read_side_effects(operand0_q);
									mask_v = (8'h01 << opcode_q[2:0]);
									alu_res_q <= target_v;
									if (opcode_q[3]) begin
										direct_write(operand0_q, target_v | mask_v);
									end else begin
										direct_write(operand0_q, target_v & ~mask_v);
									end
								end
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOV_FIXED_TO_REG: begin
								gp_write(target_reg_q, rd_op0_v);
								apply_direct_read_side_effects(operand0_q);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOV_REG_TO_FIXED: begin
								direct_write(operand0_q, gp_read(target_reg_q));
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOVW_IMM: begin
								direct_write_word({4'b0000, pair_base_q}, {operand0_q, operand1_q});
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_MOVW_FIXED: begin
								if (op_stage_q == 3'd0) begin
									target_addr_q <= operand0_q;
									op_stage_q <= 3'd1;
									state_q <= ST_OP_READ16H_SETUP;
								end else begin
									direct_write_word(target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'd0;
									enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
								end
							end

							CLASS_MEM_MOV8: begin
								if (op_stage_q == 3'd0) begin
									target_addr_q <= operand0_q;
									if (operand0_q[7:6] == 2'b10) begin
										op_stage_q <= 3'd1;
										state_q <= ST_OP_READ16H_SETUP;
									end else begin
										request_addr(AGEN_RMW, operand0_q, 16'h0000);
										op_stage_q <= 3'd2;
										state_q <= ST_MEM_READ_SETUP;
									end
								end else if (op_stage_q == 3'd1) begin
									request_addr(AGEN_RMW, target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'd2;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									gp_write({1'b0, target_addr_q[5:3]}, mem_byte_q);
									op_stage_q <= 3'd0;
									enter_core_stall(
										core_fetch_resume_stall(rmw_fetch_resume_cycles(opcode_q, target_addr_q)),
										ST_FETCH_SETUP
									);
								end
							end

							CLASS_MEM_STORE8: begin
								if (op_stage_q == 3'd0) begin
									target_addr_q <= operand0_q;
									if (operand0_q[7:6] == 2'b10) begin
										op_stage_q <= 3'b001;
										state_q <= ST_OP_READ16H_SETUP;
									end else begin
										request_addr(AGEN_RMW, operand0_q, 16'h0000);
										mem_byte_q <= source_after_rmw_update(operand0_q);
										op_stage_q <= 3'b010;
										enter_core_stall(core_fetch_resume_stall(4'd2), ST_EXECUTE);
									end
								end else if (op_stage_q == 3'b001) begin
									request_addr(AGEN_RMW, target_addr_q, {operand0_q, operand1_q});
									mem_byte_q <= source_after_rmw_update(target_addr_q);
									op_stage_q <= 3'b010;
									enter_core_stall(
										core_fetch_resume_stall(
										(target_addr_q[2:0] == 3'b000) ? 4'd1 : 4'd5
									),
										ST_EXECUTE
									);
								end else if (op_stage_q == 3'b010) begin
									if ((target_addr_q[7:6] == 2'b01) || (target_addr_q[7:6] == 2'b11)) begin
										op_stage_q <= 3'b011;
										return_state_q <= ST_EXECUTE;
									end else begin
										op_stage_q <= 3'b000;
										return_state_q <= ST_FETCH_SETUP;
									end
									state_q <= ST_MEM_WRITE_SETUP;
								end else begin
									op_stage_q <= 3'b000;
									if (target_addr_q[7:6] == 2'b01) begin
										// MOV (RR)+,R completes its pair update with the write.
										state_q <= ST_FETCH_SETUP;
									end else begin
										enter_core_stall(core_fetch_resume_stall(4'd5), ST_FETCH_SETUP);
									end
								end
							end

							CLASS_MEM_MOVW_LOAD: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									if (operand0_q[7:6] == 2'b10) begin
										op_stage_q <= 3'b001;
										state_q <= ST_OP_READ16H_SETUP;
									end else begin
										request_addr(AGEN_SMW, operand0_q, 16'h0000);
										op_stage_q <= 3'b010;
										state_q <= ST_MEM_READ_SETUP;
									end
								end else if (op_stage_q == 3'b001) begin
									request_addr(AGEN_SMW, target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'b010;
									state_q <= ST_MEM_READ_SETUP;
								end else if (op_stage_q == 3'b010) begin
									alu_res_q <= mem_byte_q;
									eff_addr_q <= eff_addr_q + 16'h0001;
									op_stage_q <= 3'b011;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									direct_write_word({4'b0000, pair_base(target_addr_q[5:3])}, {alu_res_q, mem_byte_q});
									op_stage_q <= 3'b000;
									enter_core_stall(
										core_fetch_resume_stall(smw_fetch_resume_cycles(target_addr_q)),
										ST_FETCH_SETUP
									);
								end
							end

							CLASS_MEM_MOVW_STORE: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									if (operand0_q[7:6] == 2'b10) begin
										op_stage_q <= 3'b001;
										state_q <= ST_OP_READ16H_SETUP;
									end else begin
										begin : class_mem_movw_store_stage0
											reg [15:0] source_word_v;
											request_addr(AGEN_SMW, operand0_q, 16'h0000);
											source_word_v = source_after_smw_update(operand0_q);
											alu_res_q <= source_word_v[7:0];
											mem_byte_q <= source_word_v[15:8];
										end
										op_stage_q <= 3'b010;
										return_state_q <= ST_EXECUTE;
										state_q <= ST_MEM_WRITE_SETUP;
									end
								end else if (op_stage_q == 3'b001) begin
									begin : class_mem_movw_store_stage1
										reg [15:0] source_word_v;
										request_addr(AGEN_SMW, target_addr_q, {operand0_q, operand1_q});
										source_word_v = source_after_smw_update(target_addr_q);
										alu_res_q <= source_word_v[7:0];
										mem_byte_q <= source_word_v[15:8];
									end
									op_stage_q <= 3'b010;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else if (op_stage_q == 3'b010) begin
									eff_addr_q <= eff_addr_q + 16'h0001;
									mem_byte_q <= alu_res_q;
									op_stage_q <= 3'b011;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else begin
									op_stage_q <= 3'b000;
									enter_core_stall(
										core_fetch_resume_stall(smw_fetch_resume_cycles(target_addr_q)),
										ST_FETCH_SETUP
									);
								end
							end

							CLASS_MOVW_COMPACT: begin
								if (operand0_q[7:6] == 2'b00) begin
									direct_write_word(
										{4'b0000, pair_base(operand0_q[5:3])},
										gp_read_word(pair_base(operand0_q[2:0]))
									);
								end
								op_stage_q <= 3'b000;
								enter_core_stall(core_fetch_resume_stall(4'd3), ST_FETCH_SETUP);
							end

							CLASS_MEM_CMP8: begin
								if (op_stage_q == 3'd0) begin
									target_addr_q <= operand0_q;
									if (operand0_q[7:6] == 2'b10) begin
										op_stage_q <= 3'd1;
										state_q <= ST_OP_READ16H_SETUP;
									end else begin
										request_addr(AGEN_RMW, operand0_q, 16'h0000);
										op_stage_q <= 3'd2;
										state_q <= ST_MEM_READ_SETUP;
									end
								end else if (op_stage_q == 3'd1) begin
									request_addr(AGEN_RMW, target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'd2;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									flags_cmp8(gp_read({1'b0, target_addr_q[5:3]}), mem_byte_q);
									op_stage_q <= 3'd0;
									enter_core_stall(
										core_fetch_resume_stall(rmw_fetch_resume_cycles(opcode_q, target_addr_q)),
										ST_FETCH_SETUP
									);
								end
							end

							CLASS_MEM_BYTE_ALU: begin
								if (op_stage_q == 3'd0) begin
									target_addr_q <= operand0_q;
									if (operand0_q[7:6] == 2'b10) begin
										op_stage_q <= 3'd1;
										state_q <= ST_OP_READ16H_SETUP;
									end else begin
										request_addr(AGEN_RMW, operand0_q, 16'h0000);
										op_stage_q <= 3'd2;
										state_q <= ST_MEM_READ_SETUP;
									end
								end else if (op_stage_q == 3'd1) begin
									request_addr(AGEN_RMW, target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'd2;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									exec_mem_byte_alu(opcode_q, {1'b0, target_addr_q[5:3]}, mem_byte_q);
									op_stage_q <= 3'd0;
									enter_core_stall(
										core_fetch_resume_stall(rmw_fetch_resume_cycles(opcode_q, target_addr_q)),
										ST_FETCH_SETUP
									);
								end
							end

							CLASS_FIXED_BYTE_OP: begin
								exec_fixed_byte_alu(opcode_q, operand1_q, operand0_q, rd_op1_v, rd_op0_v);
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end

							CLASS_CALL_ABS: begin
								if (op_stage_q == 3'd0) begin
									push_byte(pc_q[7:0]);
									op_stage_q <= 3'd1;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else if (op_stage_q == 3'd1) begin
									push_byte(pc_q[15:8]);
									op_stage_q <= 3'd2;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else begin
									pc_q <= {operand0_q, operand1_q};
									op_stage_q <= 3'd0;
									if (sys_q[6]) begin
										enter_core_stall(core_fetch_resume_stall(4'd3), ST_FETCH_SETUP);
									end else begin
										state_q <= ST_FETCH_SETUP;
									end
								end
							end

							CLASS_CALS: begin
								if (op_stage_q == 3'd0) begin
									vector_addr_q <= {4'h1, opcode_q[3:0], operand0_q};
									push_byte(pc_q[7:0]);
									op_stage_q <= 3'd1;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else if (op_stage_q == 3'd1) begin
									push_byte(pc_q[15:8]);
									op_stage_q <= 3'd2;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else begin
									pc_q <= vector_addr_q;
									op_stage_q <= 3'd0;
									if (sys_q[6]) begin
										enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
									end else begin
										enter_core_stall(core_fetch_resume_stall(4'd1), ST_FETCH_SETUP);
									end
								end
							end

							CLASS_MOVW_DIRECT: begin
								begin : class_movw_direct
									reg [15:0] value_v;
									value_v = word_after_sfr_reads(operand0_q, {rd_op0_v, rd_op0_hi_v}, 1'b0, 1'b0);
									apply_direct_read_word_side_effects(operand0_q);
									direct_write_word(operand1_q, value_v);
								end
								op_stage_q <= 3'b000;
								enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
							end

							CLASS_JMP_INDIRECT: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									if (!operand0_q[6]) begin
										pc_q <= gp_read_word(pair_base(operand0_q[2:0]));
										op_stage_q <= 3'b000;
										enter_core_stall(core_fetch_resume_stall(4'd3), ST_FETCH_SETUP);
									end else begin
										op_stage_q <= 3'b001;
										state_q <= ST_OP_READ16H_SETUP;
									end
								end else if (op_stage_q == 3'b001) begin
									request_addr(AGEN_ARG2, target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'b010;
									state_q <= ST_MEM_READ_SETUP;
								end else if (op_stage_q == 3'b010) begin
									alu_res_q <= mem_byte_q;
									eff_addr_q <= eff_addr_q + 16'h0001;
									op_stage_q <= 3'b011;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									pc_q <= {alu_res_q, mem_byte_q};
									op_stage_q <= 3'b000;
									if (target_addr_q[5:3] != 3'b000) begin
										enter_core_stall(core_fetch_resume_stall(4'd7), ST_FETCH_SETUP);
									end else begin
										state_q <= ST_FETCH_SETUP;
									end
								end
							end

							CLASS_CALL_INDIRECT: begin
								if (op_stage_q == 3'b000) begin
									target_addr_q <= operand0_q;
									if (!operand0_q[6]) begin
										vector_addr_q <= gp_read_word(pair_base(operand0_q[2:0]));
										push_byte(pc_q[7:0]);
										op_stage_q <= 3'b010;
										return_state_q <= ST_EXECUTE;
										state_q <= ST_MEM_WRITE_SETUP;
									end else begin
										op_stage_q <= 3'b001;
										state_q <= ST_OP_READ16H_SETUP;
									end
								end else if (op_stage_q == 3'b001) begin
									request_addr(AGEN_ARG2, target_addr_q, {operand0_q, operand1_q});
									op_stage_q <= 3'b100;
									state_q <= ST_MEM_READ_SETUP;
								end else if (op_stage_q == 3'b010) begin
									push_byte(pc_q[15:8]);
									op_stage_q <= 3'b011;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end else if (op_stage_q == 3'b011) begin
									pc_q <= vector_addr_q;
									op_stage_q <= 3'b000;
									if (target_addr_q[6] && (target_addr_q[5:3] != 3'b000)) begin
										if (sys_q[6]) begin
											enter_core_stall(core_fetch_resume_stall(4'd10), ST_FETCH_SETUP);
										end else begin
											enter_core_stall(core_fetch_resume_stall(4'd7), ST_FETCH_SETUP);
										end
									end else if (sys_q[6]) begin
										enter_core_stall(core_fetch_resume_stall(4'd6), ST_FETCH_SETUP);
									end else begin
										enter_core_stall(core_fetch_resume_stall(4'd3), ST_FETCH_SETUP);
									end
								end else if (op_stage_q == 3'b100) begin
									alu_res_q <= mem_byte_q;
									eff_addr_q <= eff_addr_q + 16'h0001;
									op_stage_q <= 3'b101;
									state_q <= ST_MEM_READ_SETUP;
								end else begin
									vector_addr_q <= {alu_res_q, mem_byte_q};
									push_byte(pc_q[7:0]);
									op_stage_q <= 3'b010;
									return_state_q <= ST_EXECUTE;
									state_q <= ST_MEM_WRITE_SETUP;
								end
							end

							CLASS_DBNZ: begin
								begin : class_dbnz
									reg [7:0] reg_v;
									reg [7:0] res_v;
									reg_v = gp_read(target_reg_q);
									res_v = reg_v - 8'h01;
									alu_res_q <= res_v;
									gp_write(target_reg_q, res_v);
									if (res_v != 8'h00) begin
										pc_q <= pc_q + {{8{operand0_q[7]}}, operand0_q};
										enter_core_stall(core_fetch_resume_stall(4'd6), ST_FETCH_SETUP);
									end else begin
										enter_core_stall(core_fetch_resume_stall(4'd2), ST_FETCH_SETUP);
									end
								end
								op_stage_q <= 3'd0;
							end

							CLASS_BR: begin
								if (cc_true(opcode_q)) begin
									pc_q <= pc_q + {{8{operand0_q[7]}}, operand0_q};
									enter_core_stall(core_fetch_resume_stall(4'd4), ST_FETCH_SETUP);
								end else begin
									state_q <= ST_FETCH_SETUP;
								end
								op_stage_q <= 3'd0;
							end

							CLASS_JMP: begin
								if (cc_true(opcode_q)) begin
									pc_q <= {operand0_q, operand1_q};
								end
								state_q <= ST_FETCH_SETUP;
								op_stage_q <= 3'd0;
							end

							default: begin
								op_stage_q <= 3'd0;
								state_q <= ST_FETCH_SETUP;
							end
						endcase
					end

					default: begin
					end
				endcase

			end
		end

		// End of beat: carry out whatever the state machine asked for.
		apply_bus_request();
		apply_addr_request();
		apply_pending_writes();
	end

endmodule
