//============================================================================
// 
//  Port to MiSTer.
//  Copyright (C) 2018 Sorgelig
//
//  Jaguar core code.
//  Copyright (C) 2018 Gregory Estrade (Torlus).
//
//  Port of Jaguar core to MiSTer (ElectronAsh / OzOnE).
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

`define AXI_DEBUG

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,					// Active-HIGH! Meaning "Low for RUNNING".
	
	input			  BTN_USER,
	input			  BTN_OSD,

	//Must be passed to hps_io module
	inout  [44:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)
	input         TAPE_IN,

	// SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

`ifdef VERILATOR
	output					os_rom_ce_n,
	output					os_rom_oe_n,
	input		[7:0]		os_rom_q,
	input						os_rom_oe,
	
	input wire        ioctl_download,
	input wire        ioctl_wr,
	//input wire [24:0] ioctl_addr,
	input wire [15:0] ioctl_data,
	input wire  [7:0] ioctl_index,
	output reg         ioctl_wait,
	
	(*noprune*)output reg [31:0] loader_addr,
	
	output wire [31:0] cart_q,
	
	output wire [1:0] cart_oe,
`endif
	
	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,
	
	input	  [6:0] USER_IN,
	output  [6:0] USER_OUT,

	output  wire         bridge_m0_waitrequest,
	output  wire [31:0]  bridge_m0_readdata,
	output  reg          bridge_m0_readdatavalid,
	input   wire [6:0]   bridge_m0_burstcount,
	input   wire [31:0]  bridge_m0_writedata,
	input   wire [19:0]  bridge_m0_address,
	input   wire         bridge_m0_write,
	input   wire         bridge_m0_read,
	input   wire         bridge_m0_byteenable,
	output  wire         bridge_m0_clk
);


assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
//assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
//assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;

//assign LED_USER  = ioctl_download;
assign LED_DISK  = 0;
assign LED_POWER = 0;



wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_33m),
	.outclk_1(clk_66m),
	.locked(pll_locked)
);

(*keep*) wire clk_sys = clk_33m;
(*keep*) wire clk_66m;


wire [1:0] scale = status[3:2];

assign VIDEO_ARX = status[1] ? 8'd16 : 8'd4;
assign VIDEO_ARY = status[1] ? 8'd9  : 8'd3; 

// Status Bit Map:
//             Uppercase O                    Lowercase o
// 0         1         2         3          4         5         6   
// 01234567890123456789012345678901 23456789012345678901234567890123
// 0123456789ABCDEFGHIJKLMNOPQRSTUV 0123456789abcdefghijklmnopqrstuv
// XXXXXXXXXXXX XXXXXXXXXXXXXXXXXXX XXXXX 

// 	"O24,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",

`include "build_id.v"
localparam CONF_STR = {
	"snes;;",
	"-;",
	"F,JAGJ64ROMBIN;",
	"-;",
	"O4,Region Setting,PAL,NTSC;",
	"O2,Cart Checksum Patch,Off,On;",
	"O1,Aspect ratio,4:3,16:9;",
	"O56,Mouse,Disabled,JoyPort1,JoyPort2;",
	"O3,CPU Speed,Normal,Turbo;",
	"-;",
	"R0,Reset;",
	"J1,A,B,C,Option,Pause,1,2,3,4,5,6,7,8,9,0,Star,Hash;",
	"J2,A,B,C,Option,Pause,1,2,3,4,5,6,7,8,9,0,Star,Hash;",
	"-;",
	"V,v1.51.",`BUILD_DATE
};


wire [63:0] status;
wire  [1:0] buttons;
wire [15:0] joystick_0;
wire [15:0] joystick_1;
wire        ioctl_download;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_data;
wire  [7:0] ioctl_index;
reg         ioctl_wait;
wire        forced_scandoubler;
wire [10:0] ps2_key;
wire [24:0] ps2_mouse;
wire [21:0] gamma_bus;

hps_io #(.STRLEN($size(CONF_STR)>>3), .PS2DIV(1000), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.status(status),
	.status_in({status[31:8],region_req,status[5:0]}),
	.status_set(region_set),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),
	.ioctl_wait(ioctl_wait),

	.ps2_key(ps2_key),
	
	.ps2_mouse(ps2_mouse),
	
	.gamma_bus(gamma_bus)
);


`ifndef VERILATOR
wire reset = RESET | status[0] | buttons[1];
`else
wire reset = RESET;
`endif


/*
gpu gpu_inst
(
	.clk( clk_sys ) ,					// input  clk
	.i_nrst( !reset & gpu_nrst ) ,// input  i_nrst
	
	.IRQRequest( IRQRequest ) ,	// output  IRQRequest
	
	.DMA_REQ( DMA_REQ ) ,			// output  DMA_REQ
	.SPUDACK( SPUDACK ) ,			// input  SPUDACK
	
	.mydebugCnt(mydebugCnt) ,		// output [31:0] mydebugCnt
	.dbg_canWrite(dbg_canWrite) ,	// output  dbg_canWrite
	
	.clkBus( clk_sys ) ,				// input  clkBus
	
	.o_command(o_command) ,			// output  o_command
	.i_busy(i_busy) ,					// input  i_busy
	.o_commandSize(o_commandSize) ,	// output [1:0] o_commandSize
	.o_write(o_write) ,				// output  o_write
	.o_adr(o_adr) ,					// output [14:0] o_adr
	.o_subadr(o_subadr) ,			// output [2:0] o_subadr
	.o_writeMask(o_writeMask) ,	// output [15:0] o_writeMask
	.i_dataIn(i_dataIn) ,			// input [255:0] i_dataIn
	.i_dataInValid(i_dataInValid) ,	// input  i_dataInValid
	.o_dataOut(o_dataOut) ,			// output [255:0] o_dataOut
	
	.gpuAdrA2(gpuAdrA2) ,			// input  gpuAdrA2
	.SPUCS(SPUCS) ,					// input  SPUCS
	.write(SWRO) ,				// input  write
	.read(SRD) ,					// input  read
	.SPU_DIN(SPU_DIN) ,			// input [31:0] SPU_DIN
	.cpuDataOut(cpuDataOut) ,		// output [31:0] cpuDataOut
	.validDataOut(validDataOut) 	// output  validDataOut
);

wire IRQRequest;
wire DMA_REQ;
reg SPUDACK;

reg gpu_nrst;

wire [31:0] mydebugCnt;
wire dbg_canWrite;

wire o_command;
wire i_busy = o_busyClient;
wire [1:0] o_commandSize;
wire o_write;
wire [14:0] o_adr;
wire [2:0] o_subadr;
wire [15:0] o_writeMask;
wire [255:0] i_dataIn = o_dataClient;
wire i_dataInValid = o_dataValidClient;
wire [255:0] o_dataOut;

reg gpuAdrA2;
wire validDataOut;
*/

assign bridge_m0_clk = clk_sys;						// output  bridge_m0_clk

wire [31:0] flag_data = {1'b0, SPUINT, SPUDREQ, 28'h0000000 };

assign bridge_m0_readdata = (bridge_m0_address[3:0]==4'h8) ? flag_data : {SPU_DOUT, SPU_DOUT};		// output [31:0] bridge_m0_readdata


reg axi_wait;
assign bridge_m0_waitrequest = axi_wait;

reg axi_readvalid;
assign bridge_m0_readdatavalid = axi_readvalid;


(*noprune*) reg [7:0] cmd_state;

always @(posedge clk_sys or posedge reset)
if (reset) begin
	//gpuAdrA2 = 1'b0;
	SPUCS = 1'b0;
	SWRO = 1'b0;
	SRD = 1'b0;
	SPU_DIN = 32'h00000000;
	
	cmd_state <= 8'd0;
	
	axi_wait <= 1'b1;
	axi_readvalid <= 1'b0;
	
	SPUDACK <= 1'b0;
	
	SPU_NRST <= 1'b1;	// Start with the GPU running.
end
else begin
	case (cmd_state)
		0: begin
			axi_wait <= 1'b0;
			axi_readvalid <= 1'b0;
			
			SPUCS <= 1'b0;
			SWRO <= 1'b0;
			SRD <= 1'b0;
			SPUDACK <= 1'b0;
			
			if (bridge_m0_write) begin
				/*case (bridge_m0_address[3:0])
					4'h0:*/ begin						// Write to GP0.
						SPU_ADDR <= bridge_m0_address >> 2;
						SPU_DIN <= bridge_m0_writedata[31:0];
						SPUCS <= 1'b1;
						SWRO <= 1'b1;
						SRD <= 1'b0;
						axi_wait <= 1'b1;
						cmd_state <= 1;
					end

					/*
					4'h4: begin						// Write to GP1.
						SPU_ADDR <= bridge_m0_address >> 2;
						SPU_DIN <= bridge_m0_writedata[31:0];
						SPUCS <= 1'b1;
						SWRO <= 1'b1;
						SRD <= 1'b0;
						axi_wait <= 1'b1;
						cmd_state <= 1;
					end

					4'h8: begin						// Write to Flags / Debug.
						SPU_ADDR <= bridge_m0_address >> 2;
						SPUCS <= 1'b0;			// No SPU write!
						SWRO <= 1'b0;
						SRD <= 1'b0;
						SPUDACK <= bridge_m0_writedata[31];
						SPU_NRST <= bridge_m0_writedata[30];
						axi_wait <= 1'b1;
						cmd_state <= 1;
					end

					4'hC: begin						// Write to SPU_DIN + SPUDACK high.
						SPU_ADDR <= bridge_m0_address >> 2;
						SPUCS <= 1'b0;			// Don't think SPUCS is required?
						SWRO <= 1'b1;
						SRD <= 1'b0;
						SPUDACK <= 1'b1;
						axi_wait <= 1'b1;
						cmd_state <= 1;
					end

					default:;
				endcase*/
			end
			else if (bridge_m0_read) begin
				/*case (bridge_m0_address[3:0])
					4'h0:*/ begin						// Read from GP0.
						SPU_ADDR <= bridge_m0_address >> 2;
						SPUCS <= 1'b1;
						SWRO <= 1'b0;
						SRD <= 1'b1;
						axi_wait <= 1'b1;
						cmd_state <= 2;
					end
					/*
					4'h4: begin						// Read from GP1.
						SPU_ADDR <= bridge_m0_address >> 2;
						SPUCS <= 1'b1;
						SWRO <= 1'b0;
						SRD <= 1'b1;
						axi_wait <= 1'b1;
						cmd_state <= 2;
					end

					4'h8: begin						// Read from Flags / Debug, without GPU select signals.
						axi_readvalid <= 1'b1;	// Pulse High for one clock (no wait for validDataOut).
						axi_wait <= 1'b0;			// Need to assert this before returning to state 0!
						cmd_state <= 0;
					end

					4'hC: begin						// Read from cpuDataOut, without GPU select signals (for DMA).
						axi_readvalid <= 1'b1;	// Pulse High for one clock (no wait for validDataOut).
						axi_wait <= 1'b0;			// Need to assert this before returning to state 0!
						cmd_state <= 0;
					end

					default:;
				endcase
				*/
			end
		end
		
		// Write
		1: begin
			SPUCS <= 1'b0;
			SWRO <= 1'b0;
			SRD <= 1'b0;
			SPUDACK <= 1'b0;
			axi_wait <= 1'b0;		// Need to assert this before returning to state 0!
			cmd_state <= 0;		// (to handle cases when axi_read or axi_write are held high.)
		end
		
		
		// Read...
		2: begin
			SPUCS <= 1'b1;
			SWRO <= 1'b0;
			SRD <= 1'b0;
			SPUDACK <= 1'b0;
			
			//if (validDataOut) begin
				axi_readvalid <= 1'b1;
				axi_wait <= 1'b0;		// Need to assert this before returning to state 0!
				cmd_state <= 0;		// (to handle cases when axi_read or axi_write are held high.)
			//end
		end
	
		default: ;
	endcase
end


(*noprune*) reg SPU_NRST;

(*noprune*) reg SPUCS;
(*noprune*) reg SRD;
(*noprune*) reg SWRO;
(*noprune*) reg [9:0] SPU_ADDR;
(*noprune*) reg [15:0] SPU_DIN;

(*keep*) wire [15:0] SPU_DOUT;
(*keep*) wire SPU_DOUT_VALID;
(*keep*) wire SPUINT;
(*keep*) wire SPUDREQ;
(*keep*) wire SPUDACK;

SPU SPU_inst
(
	.i_clk( clk_sys ) ,		// input  i_clk
	.n_rst( SPU_NRST ) ,		// input  n_rst
	
	.SPUCS( SPUCS ) ,						// input  SPUCS
	.SRD( SRD ) ,							// input  SRD
	.SWRO( SWRO ) ,						// input  SWRO
	.addr( SPU_ADDR ) ,					// input [9:0] addr
	.dataIn( SPU_DIN ) ,					// input [15:0] dataIn
	.dataOut( SPU_DOUT ) ,				// output [15:0] dataOut
	.dataOutValid( SPU_DOUT_VALID ) ,// output  dataOutValid
	
	.SPUINT( SPUINT ) ,					// output  SPUINT
	.SPUDREQ( SPUDREQ ) ,				// output  SPUDREQ
	.SPUDACK( SPUDACK ) ,				// input  SPUDACK
	
	.o_adrRAM( SPU_RAM_ADDR ) ,		// output [17:0] o_adrRAM
	.o_dataReadRAM( SPU_RAM_RD ) ,	// output  o_dataReadRAM
	.o_dataWriteRAM( SPU_RAM_WR ) ,	// output  o_dataWriteRAM
	.i_dataInRAM( SPU_RAM_DIN ) ,		// input [15:0] i_dataInRAM
	.o_dataOutRAM( SPU_RAM_DOUT ) ,	// output [15:0] o_dataOutRAM
	
	.CDRomInL( SPU_CD_INL ) ,	// input [15:0] CDRomInL
	.CDRomInR( SPU_CD_INR ) ,	// input [15:0] CDRomInR
	
	.inputL( SPU_INL ) ,		// input  inputL
	.inputR( SPU_INR ) ,		// input  inputR
	
	.AOUTL( AOUTL ) ,	// output [15:0] AOUTL
	.AOUTR( AOUTR ) ,	// output [15:0] AOUTR
	.VALIDOUT( VALIDOUT )	// output  VALIDOUT
);

wire [17:0] SPU_RAM_ADDR;
wire SPU_RAM_RD;
wire SPU_RAM_WR;
wire [15:0] SPU_RAM_DIN = sdram_dout;
wire [15:0] SPU_RAM_DOUT;

wire [15:0] SPU_CD_INL;
wire [15:0] SPU_CD_INR;

wire [15:0] SPU_INL;
wire [15:0] SPU_INR;

wire [15:0] AOUTL;
wire [15:0] AOUTR;


reg [15:0] SPU_AOUTL;
reg [15:0] SPU_AOUTR;
always @(posedge clk_sys)
if (VALIDOUT) begin
	SPU_AOUTL <= AOUTL;
	SPU_AOUTR <= AOUTR;
end

assign AUDIO_S = 1;
assign AUDIO_MIX = 0;
assign AUDIO_L = SPU_AOUTL;
assign AUDIO_R = SPU_AOUTR;


/*
wire i_command = o_command;
wire i_writeElseRead = o_write;
wire [1:0] i_commandSize = o_commandSize;
wire [14:0] i_targetAddr = o_adr;
wire [2:0] i_subAddr = o_subadr;
wire [15:0] i_writeMask = o_writeMask;
wire [255:0] i_dataClient = o_dataOut;

wire o_busyClient;
wire o_dataValidClient;
wire [255:0] o_dataClient;

PSX_DDR_Interface PSX_DDR_Interface_inst
(
	.i_clk( clk_sys ) ,						// input  i_clk
	.i_rst( !reset ) ,						// input  i_rst
	
	.i_command(i_command) ,					// input  i_command
	.i_writeElseRead(i_writeElseRead) ,	// input  i_writeElseRead
	.i_commandSize(i_commandSize) ,		// input [1:0] i_commandSize
	.i_targetAddr(i_targetAddr) ,			// input [14:0] i_targetAddr
	.i_subAddr(i_subAddr) ,					// input [2:0] i_subAddr
	.i_writeMask(i_writeMask) ,			// input [15:0] i_writeMask
	.i_dataClient(i_dataClient) ,			// input [255:0] i_dataClient
	.o_busyClient(o_busyClient) ,			// output  o_busyClient
	.o_dataValidClient(o_dataValidClient) ,	// output  o_dataValidClient
	.o_dataClient(o_dataClient) ,			// output [255:0] o_dataClient
	
	.i_busyMem(i_busyMem) ,					// input  i_busyMem
	.i_dataValidMem(i_dataValidMem) ,	// input  i_dataValidMem
	.i_dataMem(i_dataMem) ,					// input [31:0] i_dataMem
	.o_writeEnableMem(o_writeEnableMem) ,	// output  o_writeEnableMem
	.o_readEnableMem(o_readEnableMem) ,	// output  o_readEnableMem
	.o_burstLength(o_burstLength) ,		// output [7:0] o_burstLength
	.o_dataMem(o_dataMem) ,					// output [31:0] o_dataMem
	.o_targetAddr(o_targetAddr) ,			// output [25:0] o_targetAddr
	.o_byteEnableMem(o_byteEnableMem) 	// output [3:0] o_byteEnableMem
);

wire i_busyMem = DDRAM_BUSY;
wire i_dataValidMem = DDRAM_DOUT_READY;
wire [31:0] i_dataMem = DDRAM_DOUT[31:0];	// Note: DDRAM_DOUT is 64-bit.
wire o_writeEnableMem;
wire o_readEnableMem;
wire [7:0] o_burstLength;
wire [31:0] o_dataMem;
wire [25:0] o_targetAddr;
wire [3:0] o_byteEnableMem;

// From the core TO the Altera DDR controller...
assign DDRAM_CLK = clk_sys;
assign DDRAM_BURSTCNT = o_burstLength;					// (from the core TO DDR).
assign DDRAM_ADDR = {3'b001, o_targetAddr[24:0] };	// This should map the GPU Framebuffer at 0x10000000 in DDR (BYTE address!).
assign DDRAM_DIN = {32'h00000000, o_dataMem};		// Note: DDRAM_DIN is 64-bit.
assign DDRAM_BE = {4'b0000, o_byteEnableMem};		// Note: DDRAM_BE has 8 bits.
assign DDRAM_WE = o_writeEnableMem;
assign DDRAM_RD = o_readEnableMem;
*/


/*
`ifndef VERILATOR
reg [31:0] loader_addr;
`endif

//reg [15:0] loader_data;
(*keep*)wire [15:0] loader_data = ioctl_data;

reg        loader_wr;
reg        loader_en;

wire [7:0] loader_be = (loader_en && loader_addr[2:0]==0) ? 8'b11000000 :
							  (loader_en && loader_addr[2:0]==2) ? 8'b00110000 :
							  (loader_en && loader_addr[2:0]==4) ? 8'b00001100 :
							  (loader_en && loader_addr[2:0]==6) ? 8'b00000011 :
																				8'b11111111;

reg [7:0] cnt = 0;
reg [1:0] status_reg = 0;
reg       old_download;
integer   timeout = 0;


always @(posedge clk_sys or posedge reset)
if (reset) begin
	ioctl_wait <= 0;
	cnt <= 0;
	status_reg <= 0;
	old_download <= 0;
	timeout <= 0;
	loader_wr <= 0;
	loader_en <= 0;
	loader_addr <= 32'h0080_0000;
end
else begin
	old_download <= ioctl_download;
	
	loader_wr <= 0;	// Default!
	
	if (~old_download && ioctl_download && ioctl_index) begin
		loader_addr <= 32'h0080_0000;								// Force the cart ROM to load at 0x00800000 in DDR for Jag core. (byte address!)
																			// (The ROM actually gets written at 0x30800000 in DDR, which is done when load_addr gets assigned to DDRAM_ADDR below).
		loader_en <= 1;
		status_reg <= 0;
		ioctl_wait <= 0;
		timeout <= 3000000;
		cnt <= 0;
	end

	if (loader_wr) loader_addr <= loader_addr + 2;				// Writing a 16-bit WORD at a time!

	if (ioctl_wr && ioctl_index) begin
		loader_wr <= 1;
		ioctl_wait <= 1;
	end
	else if (rom_wrack) ioctl_wait <= 1'b0;
	
	//if (loader_en && DDRAM_BUSY) ioctl_wait <= 1;
	//else ioctl_wait <= 0;


//	if(ioctl_wait && !loader_wr) begin
//		if(cnt) begin
//			cnt <= cnt - 1'd1;
//			loader_wr <= 1;
//		end
//		else if(timeout) timeout <= timeout - 1;
//		else {status_reg,ioctl_wait} <= 0;
//	end


/*
	if(old_download & ~ioctl_download) begin
		loader_en <= 0;
		ioctl_wait <= 0;
	end
	if (RESET) ioctl_wait <= 0;
end
*/



/*
	.cpu_clken_dbg( cpu_clken_dbg ) ,
	
	.fx68k_addr_dbg( fx68k_addr_dbg ) ,
	
	.fx68k_as_n_dbg( fx68k_as_n_dbg ) ,
	
	.fx68k_din_dbg( fx68k_din_dbg ) ,
	.fx68k_dout_dbg( fx68k_dout_dbg ) ,
	
	.ps2_mouse( ps2_mouse ) ,
	
	.mouse_ena_1( status[6:5]==1 ) ,
	.mouse_ena_2( status[6:5]==2 )
);

wire cpu_clken_dbg;
wire fx68k_as_n_dbg;
wire [23:0] fx68k_addr_dbg;
wire [15:0] fx68k_din_dbg;
wire [15:0] fx68k_dout_dbg;
*/


wire vga_bl;
wire vga_hs_n;
wire vga_vs_n;

wire [7:0] vga_r;
wire [7:0] vga_g;
wire [7:0] vga_b;


//assign VGA_DE = !vga_bl;
//assign VGA_HS = !vga_hs_n;
//assign VGA_VS = !vga_vs_n;

assign VGA_HS = vga_hs_n ^ vga_vs_n;
assign VGA_VS = vga_vs_n;

assign VGA_R = vga_r;
assign VGA_G = vga_g;
assign VGA_B = vga_b;


assign CLK_VIDEO = clk_33m;
//wire CE_PIX = vid_ce;


//assign VGA_SL = {~interlace,~interlace} & sl[1:0];

video_mixer #(.LINE_LENGTH(640), .HALF_DEPTH(0)) video_mixer
(
	.clk_vid(CLK_VIDEO),					// input clk_sys
	.ce_pix( CE_PIX ),					// input ce_pix
	
	.ce_pix_out(CE_PIXEL),				// output ce_pix_out

	.scanlines(0),							// input [1:0] scanlines
	//.scandoubler(~interlace && (scale || forced_scandoubler)),
	
	.scandoubler(1'b0),
	
	.hq2x(scale==1),

	.mono(0),				// input mono
	
	.gamma_bus(gamma_bus),

	.R(vga_r),				// Input [DW:0] R (set by HALF_DEPTH. is [7:0] here).
	.G(vga_g),				// Input [DW:0] G (set by HALF_DEPTH. is [7:0] here).
	.B(vga_b),				// Input [DW:0] B (set by HALF_DEPTH. is [7:0] here).

	// Positive pulses.
	.HSync(vga_hs_n),		// input HSync
	.VSync(vga_vs_n),		// input VSync
	.HBlank(hblank),		// input HBlank
	.VBlank(vblank),		// input VBlank
	
//	.VGA_R( VGA_R ),		// output [7:0] VGA_R
//	.VGA_G( VGA_G ),		// output [7:0] VGA_G
//	.VGA_B( VGA_B ),		// output [7:0] VGA_B
//	.VGA_VS( VGA_VS ),	// output VGA_VS
//	.VGA_HS( VGA_HS ),	// output VGA_HS
	.VGA_DE( VGA_DE )		// output VGA_DE
);


(*keep*) wire rom_wrack = 1'b1;	// TESTING!!


/*
sdram sdram
(
	.init(~pll_locked),
	
	.clk( clk_66m ),				// Don't need the phase shift any more. DDIO is used to generate SDRAM_CLK instead (Sorg magic).

	.SDRAM_DQ( SDRAM_DQ ),		// 16 bit bidirectional data bus
	.SDRAM_A( SDRAM_A ) ,		// 13 bit multiplexed address bus
	.SDRAM_DQML( SDRAM_DQML ) ,// two byte masks
	.SDRAM_DQMH( SDRAM_DQMH ) ,// 
	.SDRAM_BA( SDRAM_BA ),		// two banks
	.SDRAM_nCS( SDRAM_nCS ),	// a single chip select
	.SDRAM_nWE( SDRAM_nWE ),	// write enable
	.SDRAM_nRAS( SDRAM_nRAS ),	// row address select
	.SDRAM_nCAS( SDRAM_nCAS ),	// columns address select
	.SDRAM_CKE( SDRAM_CKE ),	// clock enable
	.SDRAM_CLK( SDRAM_CLK ),	// clock for chip
	
	// Port 1.
//	.ch1_addr( {2'b00, ioctl_addr[24:1]} ),	// 16-bit WORD address!! [26:1]
//	.ch1_dout(  ),										// output [63:0]
//	.ch1_rnw( 1'b0 ),									// Write-only for cart loading.
//	.ch1_be( 8'b11111111 ),							// Byte enable (bits [7:0]) for 64-bit burst writes. TODO
//	.ch1_din( {ioctl_data[7:0], ioctl_data[15:8]} ),		// input [15:0]	- Data from HPS is BYTE swapped!
//	.ch1_req( ioctl_download & ioctl_wr & ioctl_index>0 ),	
//	.ch1_ready( rom_wrack ),

	// Port 1.
//	.ch1_addr( ch1_addr ),							// 64-bit WORD address. Burst Length=4. On 64-bit boundaries when the lower two bits are b00!!
//	.ch1_dout( ch1_dout ),							// output [63:0]
//	.ch1_rnw( ch1_rnw ),								// Read when HIGH. Write when LOW.
//	.ch1_be( ch1_be ),								// Byte enable (bits [7:0]) for 64-bit burst writes.
//	.ch1_din( ch1_din ),								// input [63:0]
//	.ch1_req( ch1_rd_req | ch1_wr_req ),	
//	.ch1_ready( ch1_ready ),
	
	// Port 2.
//	.ch2_addr( sdram_word_addr ),					// 16-bit WORD address!! [26:1]
//	.ch2_dout( sdram_dout ),						// output [31:0]
//	.ch2_rnw( 1'b1 ),									// Read-only for cart ROM reading!
//	.ch2_din( 16'h0000 ),							// input [15:0]
//	.ch2_req( !ioctl_download & cart_rd_trig ),
//	.ch2_ready( sdram_ready ),
	
	// Port 3.
	.ch3_addr( ch3_addr ),						// 16-bit WORD address!! [26:1]
	.ch3_dout( ch3_dout ),						// output [15:0]
	.ch3_rnw( ch3_rnw ),
	.ch3_din( ch3_din ),							// input [15:0]
	.ch3_req( ch3_req ),
	.ch3_ready( ch3_ready )
);
*/

sdram sdram
(
   .init( ~pll_locked ) ,			// reset to initialize RAM
   .clk( clk_66m ) ,					// clock ~100MHz

											// SDRAM_* - signals to the MT48LC16M16 chip
   .SDRAM_DQ( SDRAM_DQ ) ,			// [15:0] 16 bit bidirectional data bus
   .SDRAM_A( SDRAM_A ) ,			// [12:0] 13 bit multiplexed address bus
   .SDRAM_DQML( SDRAM_DQML ) ,	// two byte masks
   .SDRAM_DQMH( SDRAM_DQMH ) ,	// 
   .SDRAM_BA( SDRAM_BA ) ,			// [1:0] two banks
   .SDRAM_nCS( SDRAM_nCS ) ,		// a single chip select
   .SDRAM_nWE( SDRAM_nWE ) ,		// write enable
   .SDRAM_nRAS( SDRAM_nRAS ) ,	// row address select
   .SDRAM_nCAS( SDRAM_nCAS ) ,	// columns address select
   .SDRAM_CKE( SDRAM_CKE ) ,		// clock enable
	.SDRAM_CLK( SDRAM_CLK ) ,		// Clock.

	.wtbt( 2'b11 ) ,					// [1:0] byte enables. Active-HIGH!
	
   .addr( sdram_addr ) ,			// [23:0] 25 bit address for 8bit mode. addr[0] = 0 for 16bit mode for correct operations.
   .dout( sdram_dout ) ,			// [15:0] data output to cpu
   .din( sdram_din ) ,				// [15:0] data input from cpu
   .we( sdram_we ) ,					// cpu requests write
   .rd( sdram_rd ) ,					// cpu requests read
   .ready( sdram_ready )			// dout is valid. Ready to accept new read/write.
);

(*keep*) wire [23:0] sdram_addr = {7'b0000000, SPU_RAM_ADDR};
(*keep*) wire [15:0] sdram_din = SPU_RAM_DOUT;

(*keep*) wire [15:0] sdram_dout;

(*keep*) wire sdram_we = SPU_RAM_WR;
(*keep*) wire sdram_rd = SPU_RAM_RD;

(*keep*) wire sdram_ready;	 // todo?


endmodule
