`define MAX_ADDR 26'h3ffffff
`define BIG_BURST_LENGTH 8'h08
`define QW_BURST_LENGTH 8'h02
`define WAIT_STATE 2'b00
`define WRITE_STATE 2'b01
`define READ_REQUEST_STATE 2'b10
`define READ_STATE 2'b11
`define DOUBLE_WORD 2'd2  // 32-bit mode
`define QUAD_WORD 2'd0  // 64-bit mode
`define BIG_WORD 2'd1  // 256-bit mode

module PSX_DDR_Interface(
  // Global Connections
  input i_clk,
  input i_rst,
  
  // Client (PSX) Connections
  input i_command,  // 0 = do nothing, 1 = read/write
  input i_writeElseRead,  // 0 = read, 1 = write
  input [1:0] i_commandSize,
  input [14:0] i_targetAddr,
  input [2:0] i_subAddr,
  input [15:0] i_writeMask,
  input [255:0] i_dataClient,
  output wire o_busyClient,
  output reg o_dataValidClient,
  output reg [255:0] o_dataClient,
  
  // DDR (Memory) Connections
  input i_busyMem,
  input i_dataValidMem,
  input [31:0] i_dataMem,
  output wire o_writeEnableMem,
  output wire o_readEnableMem,
  output reg [7:0] o_burstLength,
  output reg [31:0] o_dataMem,
  output reg [25:0] o_targetAddr,
  output reg [3:0] o_byteEnableMem
);

// Internal wires
wire s_targetAddrValid;
wire [7:0] s_burstLength;
wire s_readRequestPending;  // 1 == There is a read request waiting to be started by the DDR,
// 0 == There are no active requests or there is one active request but no others are waiting to be served after its fulfillment

// Internal registers
reg [1:0] rwState;
reg [7:0] burstCounter;
reg [1:0] commandSize;
reg [15:0] writeMask;
reg [7:0] readRequestCounter;

// Output port combinational assignments
assign o_writeEnableMem = (rwState == `WRITE_STATE && i_rst == 1'b1) ? 1'b1 : 1'b0;
assign o_readEnableMem = (rwState == `READ_REQUEST_STATE && i_rst == 1'b1) ? 1'b1 : 1'b0;
assign o_busyClient = (rwState != `WAIT_STATE) ? 1'b1 : 1'b0;

// Internal signal combinational assignments
assign s_readRequestPending = (readRequestCounter == 8'h00) ? 1'b0 : 
                              (readRequestCounter == 8'h01) ? 1'b0 : // If counter == 1, treat as if no request pending 
                              1'b1;
// because the currently-active request would not yet be fulfilled. counter == 0 obviously means no request pending
assign s_targetAddrValid = (i_targetAddr <= `MAX_ADDR-(o_burstLength-8'h01)) ? 1'b1 : 1'b0;
assign s_burstLength = (i_commandSize == `BIG_WORD) ? `BIG_BURST_LENGTH :  // Write 256 bits in eight 32-bit bursts
                       (i_commandSize == `QUAD_WORD) ? `QW_BURST_LENGTH :  // Write 64 bits in two 32-bit bursts
                       (i_commandSize == `DOUBLE_WORD) ? 8'h01 :  // 32-bit write, single burst
                       `BIG_BURST_LENGTH;  // Should never occur, treat as 256-bit case

// State machine logic
always @ (posedge i_clk or negedge i_rst)
begin
  if(!i_rst)
  begin
    // Reset stuff
    commandSize <= 2'b00;
    writeMask <= 16'hFFFF;
    o_targetAddr <= 26'h0;
    burstCounter <= 8'h00;
    o_burstLength <= 8'h00;
    rwState <= `WAIT_STATE;
  end
  else
  begin
    case(rwState)
      `WAIT_STATE:begin
        if (i_command & s_targetAddrValid)
          // Able to read/write
          begin
            // Capture read/write parameters so they don't change mid-read or mid-write burst
            
            // Capture bit mode (64-bit vs 256-bit)
            commandSize <= i_commandSize;
            // Capture write mask
            writeMask <= i_writeMask;
            // Capture target address
            o_targetAddr[17:0] <= {i_targetAddr, i_subAddr};  // {15 bits, 3 bits}
            o_targetAddr[25:18] <= 8'h00;
            // Capture burst length
            o_burstLength <= s_burstLength;
        
            if (i_writeElseRead)
              // Write burst to DDR
              rwState <= `WRITE_STATE;
            else
              // Read from DDR
              rwState <= `READ_REQUEST_STATE;
          end
        else
          // Delay until read/write is triggered
          rwState <= rwState;
      end  
      
      `WRITE_STATE:begin
        // Write to DDR
        
        // Check if DDR is busy
        if (!i_busyMem)
        // DDR is ready and will queue a write request this cycle
        begin
          if (burstCounter == o_burstLength-8'h01)
          // write burst complete
          begin
            burstCounter <= 8'h00;
            rwState <= `WAIT_STATE;
          end
          else
            // write burst in progress, keep writing
            burstCounter <= burstCounter + 8'h01;
        end
        else
          // DDR is busy and will not queue a write request this cycle
          burstCounter <= burstCounter;
      end
      
      `READ_REQUEST_STATE:begin
        if(!i_busyMem)
        begin
          // Read request queued this cycle, move on to read burst
          rwState <= `READ_STATE;
        end
        else
        begin
          // Read request not queued
          rwState <= rwState;
        end
      end
      
      `READ_STATE:begin
        // Read from DDR
        if (i_dataValidMem)
          // Read burst starts on this cycle
          if (burstCounter == o_burstLength-8'h01)
          // write burst complete
          begin
            burstCounter <= 8'h00;  // Reset burst counter for next request
            if (s_readRequestPending)
              // Serve the next read request
              rwState <= `READ_STATE;
            else
              // Wait for the client to request another action
              rwState <= `WAIT_STATE;
          end
          else
            // write burst in progress, keep writing
            burstCounter <= burstCounter + 8'h01;
        else
          // Delay until DDR is free
          rwState <= rwState;
      end
      
      default:begin
        rwState <= `WAIT_STATE;
      end
    endcase
  end
end

// Track the number of pending read request
always @ (posedge i_clk or negedge i_rst)
begin
  if (!i_rst)
  begin
    readRequestCounter <= 8'h0;
  end
  else
  begin
    if (o_readEnableMem == 1'b1 && i_busyMem == 1'b0 && o_dataValidClient == 1'b0)
      // A read request will be queued on this cycle, no request fulfilled
      readRequestCounter <= readRequestCounter + 8'h01;
    else if ( (o_readEnableMem == 1'b0 || i_busyMem == 1'b1) && o_dataValidClient == 1'b1)
      // A read request will be fulfilled on this cycle, no request queued
      readRequestCounter <= readRequestCounter - 8'h01;
    else
      // A request is queued and another request is fulfilled, or neither event has occurred
      readRequestCounter <= readRequestCounter;
  end
end

// Determine data read from memory
always @ (posedge i_clk or negedge i_rst)
begin
  if (!i_rst)
  begin
    // Reset
    o_dataClient <= 256'h0;
    o_dataValidClient <= 1'b0;
  end
  else
  begin
    case (commandSize)
      `DOUBLE_WORD:begin
        //32-bit mode
        
        // Set output client data
        if (i_dataValidMem && rwState == `READ_STATE)
        begin
          o_dataClient[31:0] <= i_dataMem[31:0];
          o_dataValidClient <= 1'b1;
        end
        else
        begin
          o_dataClient[31:0] <= o_dataClient[31:0];
          o_dataValidClient <= 1'b0;
        end
        
        o_dataClient[255:32] <= 224'h0;
      end
    
      `QUAD_WORD:begin
        // 64-bit mode
        
        if (i_dataValidMem)
        begin
          case (burstCounter)
            8'h00:begin
              o_dataClient[31:0] <= i_dataMem[31:0];
              o_dataClient[255:32] <= o_dataClient[255:32];
              o_dataValidClient <= 1'b0;
            end
            
            8'h01:begin
              o_dataClient[31:0] <= o_dataClient[31:0];
              o_dataClient[63:32] <= i_dataMem[31:0];
              o_dataClient[255:64] <= o_dataClient[255:64];
              o_dataValidClient <= 1'b1;
            end
            
            default:begin
              // This case should never occur
              o_dataClient <= o_dataClient;
              o_dataValidClient <= 1'b0;
            end

          endcase
        end
        else
        begin
          o_dataClient <= o_dataClient;
          o_dataValidClient <= 1'b0;
        end
      end
      
      `BIG_WORD:begin
        // 256-bit mode
      
        if (i_dataValidMem)
        begin
          case (burstCounter)
            8'h00:begin
              o_dataClient[31:0] <= i_dataMem[31:0];
              o_dataClient[255:32] <= o_dataClient[255:32];
              o_dataValidClient <= 1'b0;
            end
            
            8'h01:begin
              o_dataClient[31:0] <= o_dataClient[31:0];
              o_dataClient[63:32] <= i_dataMem[31:0];
              o_dataClient[255:64] <= o_dataClient[255:64];
              o_dataValidClient <= 1'b0;
            end
            
            8'h02:begin
              o_dataClient[63:0] <= o_dataClient[63:0];
              o_dataClient[95:64] <= i_dataMem[31:0];
              o_dataClient[255:96] <= o_dataClient[255:96];
              o_dataValidClient <= 1'b0;
            end
            
            8'h03:begin
              o_dataClient[95:0] <= o_dataClient[95:0];
              o_dataClient[127:96] <= i_dataMem[31:0];
              o_dataClient[255:128] <= o_dataClient[255:128];
              o_dataValidClient <= 1'b0;
            end
            
            8'h04:begin
              o_dataClient[127:0] <= o_dataClient[127:0];
              o_dataClient[159:128] <= i_dataMem[31:0];
              o_dataClient[255:160] <= o_dataClient[255:160];
              o_dataValidClient <= 1'b0;
            end
            
            8'h05:begin
              o_dataClient[159:0] <= o_dataClient[159:0];
              o_dataClient[191:160] <= i_dataMem[31:0];
              o_dataClient[255:192] <= o_dataClient[255:192];
              o_dataValidClient <= 1'b0;
            end
            
            8'h06:begin
              o_dataClient[191:0] <= o_dataClient[191:0];
              o_dataClient[223:192] <= i_dataMem[31:0];
              o_dataClient[255:224] <= o_dataClient[255:224];
              o_dataValidClient <= 1'b0;
            end
            
            8'h07:begin
              o_dataClient[223:0] <= o_dataClient[223:0];
              o_dataClient[255:224] <= i_dataMem[31:0];
              o_dataValidClient <= 1'b1;  // Completed read burst
            end
            
            default:begin
              // This should never occur
              o_dataClient <= o_dataClient;
              o_dataValidClient <= 1'b0;
            end
          endcase
        end
        else
        begin
          o_dataClient <= o_dataClient;
          o_dataValidClient <= 1'b0;
        end
      end
      
      default:begin
        // This case should never occur, unused
        
        // Treat as 256-bit mode
        if (i_dataValidMem)
        begin
          case (burstCounter)
            8'h00:begin
              o_dataClient[31:0] <= i_dataMem[31:0];
              o_dataClient[255:32] <= o_dataClient[255:32];
              o_dataValidClient <= 1'b0;
            end
            
            8'h01:begin
              o_dataClient[31:0] <= o_dataClient[31:0];
              o_dataClient[63:32] <= i_dataMem[31:0];
              o_dataClient[255:64] <= o_dataClient[255:64];
              o_dataValidClient <= 1'b0;
            end
            
            8'h02:begin
              o_dataClient[63:0] <= o_dataClient[63:0];
              o_dataClient[95:64] <= i_dataMem[31:0];
              o_dataClient[255:96] <= o_dataClient[255:96];
              o_dataValidClient <= 1'b0;
            end
            
            8'h03:begin
              o_dataClient[95:0] <= o_dataClient[95:0];
              o_dataClient[127:96] <= i_dataMem[31:0];
              o_dataClient[255:128] <= o_dataClient[255:128];
              o_dataValidClient <= 1'b0;
            end
            
            8'h04:begin
              o_dataClient[127:0] <= o_dataClient[127:0];
              o_dataClient[159:128] <= i_dataMem[31:0];
              o_dataClient[255:160] <= o_dataClient[255:160];
              o_dataValidClient <= 1'b0;
            end
            
            8'h05:begin
              o_dataClient[159:0] <= o_dataClient[159:0];
              o_dataClient[191:160] <= i_dataMem[31:0];
              o_dataClient[255:192] <= o_dataClient[255:192];
              o_dataValidClient <= 1'b0;
            end
            
            8'h06:begin
              o_dataClient[191:0] <= o_dataClient[191:0];
              o_dataClient[223:192] <= i_dataMem[31:0];
              o_dataClient[255:224] <= o_dataClient[255:224];
              o_dataValidClient <= 1'b0;
            end
            
            8'h07:begin
              o_dataClient[223:0] <= o_dataClient[223:0];
              o_dataClient[255:224] <= i_dataMem[31:0];
              o_dataValidClient <= 1'b1;  // Completed read burst
            end
            
            default:begin
              // This should never occur
              o_dataClient <= o_dataClient;
              o_dataValidClient <= 1'b0;
            end
          endcase
        end
        else
        begin
          o_dataClient <= o_dataClient;
          o_dataValidClient <= 1'b0;
        end
      end
    endcase
  end
end

// Determine data to write to memory
always @ (posedge i_clk or negedge i_rst)
begin
  if (!i_rst)
  begin
    // Reset
    o_byteEnableMem <= 4'hF;
    o_dataMem <= 32'h0;
  end
  else
  begin
    if (rwState == `READ_STATE || o_readEnableMem)
    begin
      o_byteEnableMem <= 4'hF;  // Intel recommends setting all byte enable bits high when reading
    end
    else
    begin
      if (!i_busyMem)
        begin
          case (commandSize)
            `DOUBLE_WORD:begin
              // 32-bit mode
            
              // Set byte enable
              o_byteEnableMem[0] <= i_writeMask[0];
              o_byteEnableMem[1] <= i_writeMask[0];
              o_byteEnableMem[2] <= i_writeMask[1];
              o_byteEnableMem[3] <= i_writeMask[1];
              
              // Set output data to write to memory
              o_dataMem[31:0] <= i_dataClient[31:0];
            end
            
            `QUAD_WORD:begin
              // 64-bit mode
              // Will write 64 bits in two 32-bit bursts
              
              case (burstCounter)
                8'h00:begin
                  if (o_writeEnableMem == 1'b0)
                  // Waiting for a write burst, store the first burst cycle's data
                  begin
                    // Prepare to write input bits 31:0
                    
                    // Set byte enable
                    o_byteEnableMem[0] <= i_writeMask[0];
                    o_byteEnableMem[1] <= i_writeMask[0];
                    o_byteEnableMem[2] <= i_writeMask[1];
                    o_byteEnableMem[3] <= i_writeMask[1];
                    
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[31:0];
                  end
                  else
                  // Write burst begins on this cycle, store data for the 2nd cycle of the write burst
                  begin
                    // Prepare to write input bits 63:32
                    
                    // Set byte enable
                    o_byteEnableMem[0] <= writeMask[2];
                    o_byteEnableMem[1] <= writeMask[2];
                    o_byteEnableMem[2] <= writeMask[3];
                    o_byteEnableMem[3] <= writeMask[3];
                    
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[63:32];

                  end
                end
                
                8'h01:begin
                  // End of write burst, prepare for next burst
                  
                  // Set byte enable
                    o_byteEnableMem[0] <= writeMask[0];
                    o_byteEnableMem[1] <= writeMask[0];
                    o_byteEnableMem[2] <= writeMask[1];
                    o_byteEnableMem[3] <= writeMask[1];
                    
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[31:0];
                end
                
                default:begin
                  // This should never occur
                  o_dataMem[31:0] <= 32'h0;
                end
              endcase
            end
            
            `BIG_WORD:begin
              // 256-bit mode
              // Will write 256 bits in eight 32-bit bursts
              
              case (burstCounter)
                4'h0:begin
                  if (o_writeEnableMem == 1'b0)
                  // Waiting for a write burst, store the first burst cycle's data
                  begin
                    // Prepare to write input bits 31:0
                  
                    // Set byte enable
                    o_byteEnableMem[0] <= i_writeMask[0];
                    o_byteEnableMem[1] <= i_writeMask[0];
                    o_byteEnableMem[2] <= i_writeMask[1];
                    o_byteEnableMem[3] <= i_writeMask[1];
                
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[31:0];
                  end
                  else
                  // Write burst begins on this cycle, store data for the 2nd cycle of the write burst
                  begin
                    // Prepare to write input bits 63:32
                    
                    // Set byte enable
                    o_byteEnableMem[0] <= writeMask[2];
                    o_byteEnableMem[1] <= writeMask[2];
                    o_byteEnableMem[2] <= writeMask[3];
                    o_byteEnableMem[3] <= writeMask[3];
                
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[63:32];
                  end
                  
                end
              
                4'h1:begin
                  // Prepare to write input bits 95:64
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[4];
                  o_byteEnableMem[1] <= writeMask[4];
                  o_byteEnableMem[2] <= writeMask[5];
                  o_byteEnableMem[3] <= writeMask[5];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[95:64];
                end
                
                4'h2:begin
                  // Prepare to write input bits 127:96
              
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[6];
                  o_byteEnableMem[1] <= writeMask[6];
                  o_byteEnableMem[2] <= writeMask[7];
                  o_byteEnableMem[3] <= writeMask[7];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[127:96];
                end
                
                4'h3:begin
                  // Prepare to write input bits 159:128
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[8];
                  o_byteEnableMem[1] <= writeMask[8];
                  o_byteEnableMem[2] <= writeMask[9];
                  o_byteEnableMem[3] <= writeMask[9];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[159:128];
                end
                
                4'h4:begin
                  // Prepare to write input bits 191:160
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[10];
                  o_byteEnableMem[1] <= writeMask[10];
                  o_byteEnableMem[2] <= writeMask[11];
                  o_byteEnableMem[3] <= writeMask[11];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[191:160];
                end
                
                4'h5:begin
                  // Prepare to write input bits 223:192
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[12];
                  o_byteEnableMem[1] <= writeMask[12];
                  o_byteEnableMem[2] <= writeMask[13];
                  o_byteEnableMem[3] <= writeMask[13];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[223:192];
                end
                
                4'h6:begin
                  // Prepare to write input bits 255:224
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[14];
                  o_byteEnableMem[1] <= writeMask[14];
                  o_byteEnableMem[2] <= writeMask[15];
                  o_byteEnableMem[3] <= writeMask[15];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[255:224];
                end
                
                4'h7:begin
                  // End of write burst, prepare for next burst
                
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[0];
                  o_byteEnableMem[1] <= writeMask[0];
                  o_byteEnableMem[2] <= writeMask[1];
                  o_byteEnableMem[3] <= writeMask[1];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[31:0];
                end
                
                default:begin
                  // This case should never occur, indicates an error
                  o_dataMem[31:0] <= 32'h0;
                end
              endcase
            end
            
            default:begin
              // This case should never occur, unused
              // Treat same as 256-bit mode case
              
              case (burstCounter)
                4'h0:begin
                  if (o_writeEnableMem == 1'b0 || i_busyMem)
                  // Waiting for a write burst, store the first burst cycle's data
                  begin
                    // Prepare to write input bits 31:0
                  
                    // Set byte enable
                    o_byteEnableMem[0] <= i_writeMask[0];
                    o_byteEnableMem[1] <= i_writeMask[0];
                    o_byteEnableMem[2] <= i_writeMask[1];
                    o_byteEnableMem[3] <= i_writeMask[1];
                
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[31:0];
                  end
                  else
                  // Write burst begins on this cycle, store data for the 2nd cycle of the write burst
                  begin
                    // Prepare to write input bits 63:32
                    
                    // Set byte enable
                    o_byteEnableMem[0] <= writeMask[2];
                    o_byteEnableMem[1] <= writeMask[2];
                    o_byteEnableMem[2] <= writeMask[3];
                    o_byteEnableMem[3] <= writeMask[3];
                
                    // Set output data to write to memory
                    o_dataMem[31:0] <= i_dataClient[63:32];
                  end
                  
                end
              
                4'h1:begin
                  // Prepare to write input bits 95:64
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[4];
                  o_byteEnableMem[1] <= writeMask[4];
                  o_byteEnableMem[2] <= writeMask[5];
                  o_byteEnableMem[3] <= writeMask[5];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[95:64];
                end
                
                4'h2:begin
                  // Prepare to write input bits 127:96
              
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[6];
                  o_byteEnableMem[1] <= writeMask[6];
                  o_byteEnableMem[2] <= writeMask[7];
                  o_byteEnableMem[3] <= writeMask[7];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[127:96];
                end
                
                4'h3:begin
                  // Prepare to write input bits 159:128
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[8];
                  o_byteEnableMem[1] <= writeMask[8];
                  o_byteEnableMem[2] <= writeMask[9];
                  o_byteEnableMem[3] <= writeMask[9];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[159:128];
                end
                
                4'h4:begin
                  // Prepare to write input bits 191:160
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[10];
                  o_byteEnableMem[1] <= writeMask[10];
                  o_byteEnableMem[2] <= writeMask[11];
                  o_byteEnableMem[3] <= writeMask[11];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[191:160];
                end
                
                4'h5:begin
                  // Prepare to write input bits 223:192
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[12];
                  o_byteEnableMem[1] <= writeMask[12];
                  o_byteEnableMem[2] <= writeMask[13];
                  o_byteEnableMem[3] <= writeMask[13];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[223:192];
                end
                
                4'h6:begin
                  // Prepare to write input bits 255:224
                  
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[14];
                  o_byteEnableMem[1] <= writeMask[14];
                  o_byteEnableMem[2] <= writeMask[15];
                  o_byteEnableMem[3] <= writeMask[15];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[255:224];
                end
                
                4'h7:begin
                  // End of write burst, prepare for next burst
                
                  // Set byte enable
                  o_byteEnableMem[0] <= writeMask[0];
                  o_byteEnableMem[1] <= writeMask[0];
                  o_byteEnableMem[2] <= writeMask[1];
                  o_byteEnableMem[3] <= writeMask[1];
              
                  // Set output data to write to memory
                  o_dataMem[31:0] <= i_dataClient[31:0];
                end
                
                default:begin
                  // This case should never occur, indicates an error
                  o_dataMem[31:0] <= 32'h0;
                end
              endcase
            end
          endcase
        end
        else
        begin
          o_byteEnableMem[3:0] <= o_byteEnableMem[3:0];
          o_dataMem[31:0] <= o_dataMem[31:0];
        end
    end
  end
end

endmodule
