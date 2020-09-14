module i2c_top(

  input clk,
  input reset,
    
  input [2:0] reg_addr,
  input [7:0] data_i,   // reg data
  input [7:0] txr,      // transmit data

  input sda_pad_i,

  output top_en,      // core enable signal
  output prer_en,

  output sda_pad_o,
  output SCL,
  output sda_pad
);

  wire reg_addr;    // register address

  wire sda_pad_i, sda_pad_o;
  wire state_end;   // not get ack signal from slave, then community end
  wire top_rxr; 

  wire SCL;

  wire [7:0]   txr_test;    /// test

  reg top_en, prer_en;

  reg [15:0]  prer;        // CLK DIVIDE  
  reg [7:0]   rxr;         // RECIVE register
  reg [7:0]   cr;          // COMMAND REGISTE
  reg [7:0]   sr;          // STATUS REGISTER
  reg [7:0]   addr;        // ADDRESE REGISTER
 
  wire        clear;           // clear command register

  always @(posedge clk) begin
    if(reset) begin
      top_en  <= 0;
      prer_en <= 1;
      prer    <= 0;
      cr      <= 0;
      sr      <= 0;
    end
    else if(state_end & clear) begin
      top_en    <= 0;
      prer_en   <= 1;
      cr        <= 0;
      prer      <= 0;     
    end
    else begin
      case(reg_addr)
      3'b001:
        prer [7:0] <= data_i;
      3'b010: begin
        prer [15:8] <= data_i;
        prer_en     <= 0;
      end
      3'b011:
        cr <= data_i;
      3'b100:
        addr <= data_i;
      3'b101:
        top_en <= 1'b1;     // core enable signal
      3'b110:
        top_en <= 1'b0;
      endcase
    end        
  end

  wire start  = cr[7];
  wire stop   = cr[6];
  wire write  = cr[5];
  wire read   = cr[4];
  wire ack_m  = cr[3];        // ack to slave
  wire ack_s  = cr[2];        // ack from slave

  ////////////// recive ack ///////////////////
  always @(posedge clk) begin
    if(ack_chk) cr[2] <= ~sda_pad_i;
    else cr[2] = 0;
  end
  ///////////////////////////////////////////// 

  wire get_data; // save rxr

  //////// recive data /////////////////
  always @(posedge clk) begin
    if(reset) rxr <= 0;
    else if(get_data) rxr <= {rxr[6:0], sda_pad_i};
  end
  //////////////////////////////////////

  I2C_transmit transmit_dyte(
    .clk       (clk         ),
    .reset     (reset       ),
    .prer      (prer        ),
    .prer_en   (prer_en     ),
    .addr      (addr        ),
    .start     (start       ),
    .stop      (stop        ),
    .write     (write       ),
    .read      (read        ),
    .ack_chk   (ack_chk     ),
    .ack_s     (ack_s       ),
    .ack_m     (ack_m       ),
    .top_en    (top_en      ),
    .SDA_i     (sda_pad_i   ),
    .SDA_o     (sda_pad_o   ),
    .SCL       (SCL         ),
    .clear     (clear       ),
    .state_end (state_end   ),
    .sda_pad   (sda_pad     ),
    .read_en   (get_data    ),
    .txr       (txr         )
  );

endmodule // i2c_top

`define I2C_IDLE    5'b0_0000
`define I2C_START   5'b0_0001
`define I2C_ADDR    5'b0_0010
`define I2C_WRITE   5'b0_0100
`define I2C_ACK_W   5'b0_1000
`define I2C_STOP    5'b1_0000 
`define I2C_READ    5'b1_0001
`define I2C_ACK_R   5'b1_0010

module I2C_transmit(

  input clk,
  input reset,

  input [15:0]    prer, 
  input [7:0]     addr,
  input [7:0]     txr,

  input start,
  input stop,
  input write,   
  input read,

  input top_en,     // core enable signal
  input prer_en,

  input SDA_i,

  input ack_s,    // slave -> master 
  input ack_m,    // master -> slave 
  
  output state_end,

  output clk_en,
  output clear,

  output read_en,
  output ack_chk,  // when get ack signal(from slave) check

  output SCL,
  output sda_pad,
  output SDA_o    
);
  
  wire start, write, read, stop;
  wire SDA_i;

  wire prer_en;
  wire state_end;
  reg  clear;
  wire ack_chk;

  reg [7:0] sr;   // SHIFT REGISTER -> send data 1bit
  reg [7:0] ar;   // ADDRESS REGISTER

  reg [2:0] next_state;
  reg [2:0] before_state;
  reg shift, shift_a;
  
  reg  [4:0] b_state;
  reg  read_en;
  reg  txr_bit, txr_bit_a;
  wire ack_bit;    // from bit_transmit

  reg  in;
  wire in_a_d;

  reg [2:0] ack_cnt;  //  ack_chk counter
  wire      cnt;

  assign in_a_d = (in) ? 1'b1 : 1'b0; // input addr & data

  wire run = ~(start | write | read | stop);

  always @(posedge clk) begin
    if(reset) begin
      sr  <= 0;
      ar  <= 0;
    end
    else if(in_a_d) begin
      sr    <= txr;      
      ar    <= addr;
    end   
    else if(shift_a)
      ar <=  {ar[6:0], 1'b0};
    else if(shift)
      sr <=  {sr[6:0], 1'b0};
  end

  always @(posedge clk) begin
    if(reset)
      ack_cnt <= 3'h7;
    else if(in_a_d)
      ack_cnt <= 3'h7;
    else if(shift_a | shift)
      ack_cnt <=  ack_cnt - 3'h1; 
  end

  // when recive nack signal from slave
  always @(posedge SCL) begin
    if (ack_chk & ~ack_s) begin // nack signal
      case(before_state)
        ST_ADDR: 
          next_state  <=  ST_START;
      endcase    
    end
  end

  assign cnt = ~(|ack_cnt);     // ack_counter
  
  parameter [2:0] ST_IDLE     = 3'b000;
  parameter [2:0] ST_START    = 3'b001;
  parameter [2:0] ST_WRITE    = 3'b010;
  parameter [2:0] ST_ADDR     = 3'b011;
  parameter [2:0] ST_STOP     = 3'b100;
  parameter [2:0] ST_ACK      = 3'b101;
  parameter [2:0] ST_READ     = 3'b110;
    
  always @(posedge clk) begin
    if(reset)   begin
      in            <= 0;
      next_state    <= ST_IDLE;
      before_state  <= ST_IDLE;
      b_state       <= `I2C_IDLE;
    end
    else begin
      if(top_en) begin
        in          <= 0;
        shift       <= 0;
        shift_a     <= 0;
        txr_bit     <= sr[7];
        txr_bit_a   <= ar[7];
        read_en     <= 0;

        case(next_state)
        ST_IDLE:
         if(!run) begin
            if(start) begin
              next_state    <= ST_START;
              b_state       <= `I2C_START;
            end          
          end

        ST_START:   // 3'b001
          if(ack_bit) begin
            in            <=  1'b1;
            next_state    <=  ST_ADDR;
            b_state       <= `I2C_ADDR;
          end
                
        ST_ADDR:  // 3'b011
          if(ack_bit) begin
            if(cnt) begin
              next_state    <= ST_ACK;
              before_state  <= ST_ADDR;
              b_state       <=`I2C_ACK_W;
              shift_a       <= 1'b1;
            end
            else begin
              next_state    <= ST_ADDR;
              b_state       <= `I2C_ADDR;
              shift_a       <= 1'b1;
            end
          end        
            
        ST_ACK: // 3'b101
          if(ack_bit) begin
            if(!stop)begin
              if(write) begin
                in            <=  1'b1;
                next_state    <=  ST_WRITE;
                b_state       <=  `I2C_WRITE;
              end                
              else if(read) begin              
                in            <=  1'b1;
                next_state    <=  ST_READ;
                b_state       <=  `I2C_READ;
              end
            end       
            else begin
              next_state    <=  ST_STOP;
              b_state       <=  `I2C_STOP;
            end
          end
            
       ST_WRITE: // 3'b010
          if(ack_bit) begin
            if(cnt) begin
              next_state    <=  ST_ACK;
              before_state  <=  ST_WRITE;
              b_state       <= `I2C_ACK_W;
            end
            else begin
              next_state    <=  ST_WRITE;
              b_state       <=  `I2C_WRITE;
              shift         <=  1'b1;
            end
          end

        ST_READ: // 3'b110
          if(ack_bit) begin
            if(cnt) begin
              next_state    <=  ST_ACK;
              before_state  <=  ST_READ;
              b_state       <= `I2C_ACK_R;
              read_en       <=  1'b1;
            end
            else begin
              next_state    <=  ST_READ;
              b_state       <=  `I2C_READ;
              shift         <=  1'b1;
              read_en       <=  1'b1;
            end
          end                 

        ST_STOP: begin    // 3'b100
          clear         <= 1'b1;
          next_state    <= ST_STOP;
          b_state       <= `I2C_STOP;
        end
        endcase
      end
    end
  end

  bit_transmit trans_bit(
    .clk         (clk        ),
    .reset       (reset      ),
    .b_state     (b_state    ),
    .txr_bit     (txr_bit    ),
    .txr_bit_a   (txr_bit_a  ),
    .prer        (prer       ), 
    .prer_en     (prer_en    ),
    .ack_bit     (ack_bit    ),
    .ack_m       (ack_m      ),
    .ack_chk     (ack_chk    ),
    .state_end   (state_end  ),
    .SCL         (SCL        ),
    .top_en      (top_en     ),
    .sda_pad     (sda_pad    ),
    .SDA_i       (SDA_i      ),
    .SDA_o       (SDA_o      )
  );

endmodule // I2C_transmit

`define I2C_IDLE    5'b0_0000
`define I2C_START   5'b0_0001
`define I2C_ADDR    5'b0_0010
`define I2C_WRITE   5'b0_0100
`define I2C_ACK     5'b0_1000
`define I2C_STOP    5'b1_0000
`define I2C_READ    5'b1_0001

module bit_transmit(

  input clk,
  input reset,
  input txr_bit,      // trans_bit
  input txr_bit_a,    // trans_addr
  
  input top_en,
  input [15:0] prer,
  input prer_en,
  input SDA_i,

  input ack_m,

  input [4:0] b_state,

  output state_end,
  output ack_chk, 

  output ack_bit,
  output SCL,
  output sda_pad,
  output SDA_o
);
  
  wire SDA_i;
  wire clk_en, prer_en;
  wire ack_m;

  reg state_end;
  reg c_stop;    
  reg SCL;
  reg [15:0] counter;
  reg [4:0] bit_state;
  reg ack_bit, ack_chk;  

  reg SDA_o, sda_pad;  
  
  assign clk_en = (counter == 1) ? 1'b1 : 1'b0;

  always @(posedge clk) begin
    if(reset) begin            
      counter     <= 0;
      // c_stop      <= 0;
    end
    else if(prer_en) begin 
      counter <= prer;
      // c_stop  <= 1'b1;
    end
    else if(counter == 1)
      counter <= prer;
    else 
      counter <= counter - 1'b1;
    end

  parameter [4:0] idle    = 5'b0_0000;

  parameter [4:0] start_a = 5'b0_0001;
  parameter [4:0] start_b = 5'b0_0010;
  parameter [4:0] start_c = 5'b0_0011;
  parameter [4:0] start_d = 5'b0_0100;
  parameter [4:0] start_e = 5'b0_0101;  // 5
    
  parameter [4:0] addr_a  = 5'b0_0110;
  parameter [4:0] addr_b  = 5'b0_0111;
  parameter [4:0] addr_c  = 5'b0_1000;
  parameter [4:0] addr_d  = 5'b0_1001;  // 9
    
  parameter [4:0] data_a  = 5'b0_1010;
  parameter [4:0] data_b  = 5'b0_1011;
  parameter [4:0] data_c  = 5'b0_1100;
  parameter [4:0] data_d  = 5'b0_1101;  // 13
    
  parameter [4:0] stop_a  = 5'b0_1110;
  parameter [4:0] stop_b  = 5'b0_1111;
  parameter [4:0] stop_c  = 5'b1_0000; 
  parameter [4:0] stop_d  = 5'b1_0001; 
  parameter [4:0] stop_e  = 5'b1_0010;  // 18

  parameter [4:0] ack_a   = 5'b1_0011;
  parameter [4:0] ack_b   = 5'b1_0100;
  parameter [4:0] ack_c   = 5'b1_0101;
  parameter [4:0] ack_d   = 5'b1_0110;  // 22

  parameter [4:0] read_a  = 5'b1_0111;
  parameter [4:0] read_b  = 5'b1_1000;
  parameter [4:0] read_c  = 5'b1_1001;
  parameter [4:0] read_d  = 5'b1_1010; // 26

  parameter [4:0] mck_a   = 5'b1_1011;
  parameter [4:0] mck_b   = 5'b1_1100;
  parameter [4:0] mck_c   = 5'b1_1101;
  parameter [4:0] mck_d   = 5'b1_1110;  // 30

  always @(posedge clk) begin
    if(reset) begin
      state_end <= 0;
      bit_state <= idle;
      sda_pad   <= 1'b0;
      ack_chk   <= 1'b0;

    end
    else begin
      ack_bit <= 1'b0;
      
      if(clk_en & top_en) begin
        ack_chk <= 1'b0;
       
        case(bit_state)
        idle: begin 
           
          case(b_state)
            `I2C_START: bit_state <= start_a;
            `I2C_ADDR : bit_state <= addr_a;  // b0_0010
            `I2C_WRITE: bit_state <= data_a;
            `I2C_READ : bit_state <= read_a;
            `I2C_STOP : bit_state <= stop_a;
            `I2C_ACK_W: bit_state <= ack_a; // 5'b0_1000
            `I2C_ACK_R: bit_state <= mck_a; // 5'b0_1000
          endcase
        end

        start_a: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= 1'b1;
          bit_state <= start_b;
        end

        start_b: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= 1'b1;
          bit_state <= start_c;
        end

        start_c: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= 1'b0;
          bit_state <= start_d;
        end

        start_d: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b0;
          SDA_o     <= 1'b0;
          bit_state <= start_e;
        end

        start_e: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b0;
          SDA_o     <= 1'b0;
          ack_bit   <= 1'b1;
          bit_state <= idle;
        end
            
        addr_a: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b0;
          SDA_o     <= txr_bit_a;
          bit_state <= addr_b;
        end

        addr_b: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= txr_bit_a;
          bit_state <= addr_c;
        end

        addr_c: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= txr_bit_a;
          bit_state <= addr_d;
        end
           
        addr_d: begin              
          sda_pad     <= 1'b1;
          SCL         <= 1'b0;
          SDA_o       <= txr_bit_a;
          ack_bit     <= 1'b1;
          bit_state   <= idle;
        end
            
        ack_a: begin       //5'b1_0010
          sda_pad     <= 1'b0;
          SCL         <= 1'b0;
          ack_chk     <= 1'b1;         
          bit_state   <= ack_b;
        end

        ack_b: begin
          sda_pad     <= 1'b0;
          SCL         <= 1'b1;
          ack_chk     <= 1'b1;         
          bit_state   <= ack_c;
        end

        ack_c: begin
          sda_pad     <= 1'b0;
          SCL         <= 1'b1;
          ack_chk     <= 1'b1;         
          bit_state   <= ack_d;
        end

        ack_d: begin
          sda_pad   <= 1'b0;
          SCL       <= 1'b0;
          ack_bit   <= 1'b1;
          ack_chk   <= 1'b0;
          bit_state <= idle;              
        end

        mck_a: begin
          sda_pad     <= 1'b1;
          SCL         <= 1'b0;
          SDA_o       <= ack_m;
          bit_state   <= mck_b;
        end

        mck_b: begin
          sda_pad     <= 1'b1;
          SCL         <= 1'b1;
          SDA_o       <= ack_m;
          bit_state   <= mck_c;
        end

        mck_c: begin
          sda_pad     <= 1'b1;
          SCL         <= 1'b1;
          SDA_o       <= ack_m;
          bit_state   <= mck_d;
        end

        mck_d: begin
          sda_pad     <= 1'b1;
          SCL         <= 1'b0;
          SDA_o       <= ack_m;
          ack_bit     <= 1'b1;
          bit_state   <= idle;
        end
    
        data_a: begin // 8               
          sda_pad   <= 1'b1;
          SCL       <= 1'b0;
          SDA_o     <= txr_bit;
          bit_state <= data_b;
        end

        data_b: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= txr_bit;
          bit_state <= data_c;
        end

        data_c: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= txr_bit;
          bit_state <= data_d;
        end

        data_d: begin
          sda_pad     <= 1'b1;
          SCL         <= 1'b0;
          SDA_o       <= txr_bit;
          ack_bit     <= 1'b1;
          bit_state   <= idle;
        end

        read_a: begin
          sda_pad     <= 1'b0;
          SCL         <= 1'b0;
          bit_state   <= read_b;
        end

        read_b: begin
          sda_pad     <= 1'b0;
          SCL         <= 1'b1;
          bit_state   <= read_c;
        end

        read_c: begin
          sda_pad     <= 1'b0;
          SCL         <= 1'b1;
          bit_state   <= read_d;
        end

        read_d: begin
          sda_pad     <= 1'b0;
          SCL         <= 1'b0;
          ack_bit     <= 1'b1;
          bit_state   <= idle;
        end

        stop_a: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b0;
          SDA_o     <= 1'b0;
          bit_state <= stop_b;
        end

        stop_b: begin
          sda_pad   <= 1'b1;            
          SCL       <= 1'b0;
          SDA_o     <= 1'b0;
          bit_state <= stop_c;
        end

        stop_c: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= 1'b0;
          bit_state <= stop_d;
        end

        stop_d: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= 1'b0;
          bit_state <= stop_e;
        end

        stop_e: begin
          sda_pad   <= 1'b1;
          SCL       <= 1'b1;
          SDA_o     <= 1'b1;
          ack_bit   <= 1'b1;
          state_end <= 1'b1;
          bit_state <= idle;
        end
      endcase
    end
  end
end

endmodule // bit_transmit
  // ----------------------- i2cSlave_define.v --------------------

  // stream states
  `define STREAM_IDLE 2'b00
  `define STREAM_READ 2'b01
  `define STREAM_WRITE_ADDR 2'b10
  `define STREAM_WRITE_DATA 2'b11

  // start stop detection states
  `define NULL_DET 2'b00
  `define START_DET 2'b01
  `define STOP_DET 2'b10

  // i2c ack and nak
  `define I2C_NAK 1'b1
  `define I2C_ACK 1'b0

  // ----------------------------------------------------------------
  // ------------- modify constants below this line -----------------
  // ----------------------------------------------------------------

  // i2c device address
  `define I2C_ADDRESS 7'h5C

  // System clock frequency in MHz
  // If you are using a clock frequency below 24MHz, then the macro
  // for SDA_DEL_LEN will result in compile errors for i2cSlave.v
  // you will need to hand tweak the SDA_DEL_LEN constant definition
  `define CLK_FREQ 24

  // Debounce SCL and SDA over this many clock ticks
  // The rise time of SCL and SDA can be up to 1000nS (in standard mode)
  // so it is essential to debounce the inputs.
  // The spec requires 0.05V of hysteresis, but in practise
  // simply debouncing the inputs is sufficient
  // I2C spec requires suppresion of spikes of 
  // maximum duration 50nS, so this debounce time should be greater than 50nS
  // Also increases data hold time and decreases data setup time
  // during an I2C read operation
  // 10 ticks = 208nS @ 48MHz
  `define DEB_I2C_LEN (10*`CLK_FREQ)/48

  // Delay SCL for use as internal sampling clock
  // Using delayed version of SCL to ensure that 
  // SDA is stable when it is sampled.
  // Not entirely citical, as according to I2C spec
  // SDA should have a minimum of 100nS of set up time
  // with respect to SCL rising edge. But with the very slow edge 
  // speeds used in I2C it is better to err on the side of caution.
  // This delay also has the effect of adding extra hold time to the data
  // with respect to SCL falling edge. I2C spec requires 0nS of data hold time.
  // 10 ticks = 208nS @ 48MHz
  `define SCL_DEL_LEN (10*`CLK_FREQ)/48

  // Delay SDA for use in start/stop detection
  // Use delayed SDA during start/stop detection to avoid
  // incorrect detection at SCL falling edge.
  // From I2C spec start/stop setup is 600nS with respect to SCL rising edge
  // and start/stop hold is 600nS wrt SCL falling edge.
  // So it is relatively easy to discriminate start/stop,
  // but data setup time is a minimum of 100nS with respect to SCL rising edge
  // and 0nS hold wrt to SCL falling edge.
  // So the tricky part is providing robust start/stop detection
  // in the presence of regular data transitions.
  // This delay time should be less than 100nS
  // 4 ticks = 83nS @ 48MHz
  `define SDA_DEL_LEN (4*`CLK_FREQ)/48
  module i2cSlaveTop (
    CLK,
    CLR,
    sda,
    scl,
    myReg0
  );
  input CLK;
  input CLR;
  inout sda;
  input scl;
  output [7:0] myReg0;

  //reg sdaOut;
  //wire sdaIn;
  //wire sdaEn;
  //assign sda = sdaOut ? 1'bz : 1'b0;
  ///assign sdaIn = (sdaEn == 0) ? sda : 1'bz;
  //assign sdaOut = sda;

  //assign sdaIn= (Out == 1'b0) ? 1'b0 : 1'bz;
  //assign sda = (sdaEn == 1'b1 && sdaOut == 1'b0) ? 1'b0 : (sdaEn == 1'b1 && sdaOut == 1'b1)? 1'b1 : 1'bz;
  //assign sdaIn = sda;


  i2cSlave u_i2cSlave(
    .clk(CLK),
    .rst(CLR),
    .sda(sda),
    .scl(scl),
    .myReg0(myReg0),
    .myReg1(),
    .myReg2(),
    .myReg3(),
    .myReg4(8'h12),
    .myReg5(8'h34),
    .myReg6(8'h56),
    .myReg7(8'h78)
  // .sdaEn(sdaEn)
  );


  endmodule

  `define STREAM_IDLE 2'b00
  `define STREAM_READ 2'b01
  `define STREAM_WRITE_ADDR 2'b10
  `define STREAM_WRITE_DATA 2'b11

  // start stop detection states
  `define NULL_DET 2'b00
  `define START_DET 2'b01
  `define STOP_DET 2'b10

  // i2c ack and nak
  `define I2C_NAK 1'b1
  `define I2C_ACK 1'b0

  // ----------------------------------------------------------------
  // ------------- modify constants below this line -----------------
  // ----------------------------------------------------------------

  // i2c device address
  `define I2C_ADDRESS 7'h5C

  // System clock frequency in MHz
  // If you are using a clock frequency below 24MHz, then the macro
  // for SDA_DEL_LEN will result in compile errors for i2cSlave.v
  // you will need to hand tweak the SDA_DEL_LEN constant definition
  `define CLK_FREQ 24

  // Debounce SCL and SDA over this many clock ticks
  // The rise time of SCL and SDA can be up to 1000nS (in standard mode)
  // so it is essential to debounce the inputs.
  // The spec requires 0.05V of hysteresis, but in practise
  // simply debouncing the inputs is sufficient
  // I2C spec requires suppresion of spikes of 
  // maximum duration 50nS, so this debounce time should be greater than 50nS
  // Also increases data hold time and decreases data setup time
  // during an I2C read operation
  // 10 ticks = 208nS @ 48MHz
  `define DEB_I2C_LEN (10*`CLK_FREQ)/48

  // Delay SCL for use as internal sampling clock
  // Using delayed version of SCL to ensure that 
  // SDA is stable when it is sampled.
  // Not entirely citical, as according to I2C spec
  // SDA should have a minimum of 100nS of set up time
  // with respect to SCL rising edge. But with the very slow edge 
  // speeds used in I2C it is better to err on the side of caution.
  // This delay also has the effect of adding extra hold time to the data
  // with respect to SCL falling edge. I2C spec requires 0nS of data hold time.
  // 10 ticks = 208nS @ 48MHz
  `define SCL_DEL_LEN (10*`CLK_FREQ)/48

  // Delay SDA for use in start/stop detection
  // Use delayed SDA during start/stop detection to avoid
  // incorrect detection at SCL falling edge.
  // From I2C spec start/stop setup is 600nS with respect to SCL rising edge
  // and start/stop hold is 600nS wrt SCL falling edge.
  // So it is relatively easy to discriminate start/stop,
  // but data setup time is a minimum of 100nS with respect to SCL rising edge
  // and 0nS hold wrt to SCL falling edge.
  // So the tricky part is providing robust start/stop detection
  // in the presence of regular data transitions.
  // This delay time should be less than 100nS
  // 4 ticks = 83nS @ 48MHz
  `define SDA_DEL_LEN (4*`CLK_FREQ)/48


  module i2cSlave (
    clk,
    rst,
    sda,
    scl,
    myReg0,
    myReg1,
    myReg2,
    myReg3,
    myReg4,
    myReg5,
    myReg6,
    myReg7
  );

  input clk;
  input rst;
  inout sda;
  input scl;
  output [7:0] myReg0;
  output [7:0] myReg1;
  output [7:0] myReg2;
  output [7:0] myReg3;
  input [7:0] myReg4;
  input [7:0] myReg5;
  input [7:0] myReg6;
  input [7:0] myReg7;


  // local wires and regs
  reg sdaDeb;
  reg sclDeb;
  reg [`DEB_I2C_LEN-1:0] sdaPipe;
  reg [`DEB_I2C_LEN-1:0] sclPipe;

  reg [`SCL_DEL_LEN-1:0] sclDelayed;
  reg [`SDA_DEL_LEN-1:0] sdaDelayed;
  reg [1:0] startStopDetState;
  wire clearStartStopDet;
  wire sdaOut;
  wire sdaEn;
  wire sdaIn;
  wire [7:0] regAddr;
  wire [7:0] dataToRegIF;
  wire writeEn;
  wire [7:0] dataFromRegIF;
  reg [1:0] rstPipe;
  wire rstSyncToClk;
  reg startEdgeDet;

  assign sda = (sdaEn == 1) ? sdaOut : 1'bz;
  assign sdaIn = (sdaEn == 0) ? sda : 1'bz;
  //assign sdaIn = (sdaEn == 0) ? sda : 1'bz;
  //assign sdaOut = (sdaEn == 1) ? sda : 1'bz;

  //assign da = sdaOut;
  //sync rst rsing edge to clk
  always @(posedge clk) begin
    if (rst == 1'b1)
      rstPipe <= 2'b10;          ///////// 2'b11;
    else
      rstPipe <= {rstPipe[0], 1'b0};
  end

  assign rstSyncToClk = rstPipe[1];

  // debounce sda and scl
  always @(posedge clk) begin
    if (rstSyncToClk == 1'b1) begin
      sdaPipe <= {`DEB_I2C_LEN{1'b1}};
      sdaDeb <= 1'b1;
      sclPipe <= {`DEB_I2C_LEN{1'b1}};
      sclDeb <= 1'b1;
    end
    else begin
      sdaPipe <= {sdaPipe[`DEB_I2C_LEN-2:0], sdaIn};
      sclPipe <= {sclPipe[`DEB_I2C_LEN-2:0], scl};
      if (sclPipe[0] == 1'b1)
        sclDeb <= 1'b1;
      else if (sclPipe[0] == 1'b0)
        sclDeb <= 1'b0;
      if (sdaPipe[0] == 1'b1) // &sdaPipe[`DEB_I2C_LEN-1:1] == 1'b1;
        sdaDeb <= 1'b1;
      else if (sdaPipe[0] == 1'b0) // |sdaPipe[`DEB_I2C_LEN-1:1] == 1'b0;
        sdaDeb <= 1'b0;
    end
  end


  // delay scl and sda
  // sclDelayed is used as a delayed sampling clock
  // sdaDelayed is only used for start stop detection
  // Because sda hold time from scl falling is 0nS
  // sda must be delayed with respect to scl to avoid incorrect
  // detection of start/stop at scl falling edge. 
  always @(posedge clk) begin
    if (rstSyncToClk == 1'b1) begin
      sclDelayed <= {`SCL_DEL_LEN{1'b1}};
      sdaDelayed <= {`SDA_DEL_LEN{1'b1}};
    end
    else begin
      sclDelayed <= {sclDelayed[`SCL_DEL_LEN-2:0], sclDeb};
      sdaDelayed <= {sdaDelayed[`SDA_DEL_LEN-2:0], sdaDeb};
    end
  end

  // start stop detection
  always @(posedge clk) begin
    if (rstSyncToClk == 1'b1) begin
      startStopDetState <= `NULL_DET;
      startEdgeDet <= 1'b0;
    end
    else begin
      if (sclDeb == 1'b1 && sdaDelayed[`SDA_DEL_LEN-2] == 1'b0 && sdaDelayed[`SDA_DEL_LEN-1] == 1'b1)
        startEdgeDet <= 1'b1;
      else
        startEdgeDet <= 1'b0;
      if (clearStartStopDet == 1'b1)
        startStopDetState <= `NULL_DET;
      else if (sclDeb == 1'b1) begin
        if (sdaDelayed[`SDA_DEL_LEN-2] == 1'b1 && sdaDelayed[`SDA_DEL_LEN-1] == 1'b0) 
          startStopDetState <= `STOP_DET;
        else if (sdaDelayed[`SDA_DEL_LEN-2] == 1'b0 && sdaDelayed[`SDA_DEL_LEN-1] == 1'b1)
          startStopDetState <= `START_DET;
      end
    end
  end


  registerInterface u_registerInterface(
    .clk(clk),
    .addr(regAddr),
    .dataIn(dataToRegIF),
    .writeEn(writeEn),
    .dataOut(dataFromRegIF),
    .myReg0(myReg0),
    .myReg1(myReg1),
    .myReg2(myReg2),
    .myReg3(myReg3),
    .myReg4(myReg4),
    .myReg5(myReg5),
    .myReg6(myReg6),
    .myReg7(myReg7)
  );

  serialInterface u_serialInterface (
    .clk(clk), 
    .rst(rstSyncToClk), 
    .dataIn(dataFromRegIF), 
    .dataOut(dataToRegIF), 
    .writeEn(writeEn),
    .regAddr(regAddr), 
    .scl(sclDelayed[`SCL_DEL_LEN-1]), 
    .sdaIn(sdaDeb), 
    .sdaOut(sdaOut), 
    .sdaEn(sdaEn),
    .startStopDetState(startStopDetState),
    .clearStartStopDet(clearStartStopDet) 
  );

  endmodule

  // ----------------------- i2cSlave_define.v --------------------

  // stream states
  `define STREAM_IDLE 2'b00
  `define STREAM_READ 2'b01
  `define STREAM_WRITE_ADDR 2'b10
  `define STREAM_WRITE_DATA 2'b11

  // start stop detection states
  `define NULL_DET 2'b00
  `define START_DET 2'b01
  `define STOP_DET 2'b10

  // i2c ack and nak
  `define I2C_NAK 1'b1
  `define I2C_ACK 1'b0

  // ----------------------------------------------------------------
  // ------------- modify constants below this line -----------------
  // ----------------------------------------------------------------

  // i2c device address
  `define I2C_ADDRESS 7'h5C

  // System clock frequency in MHz
  // If you are using a clock frequency below 24MHz, then the macro
  // for SDA_DEL_LEN will result in compile errors for i2cSlave.v
  // you will need to hand tweak the SDA_DEL_LEN constant definition
  `define CLK_FREQ 24

  // Debounce SCL and SDA over this many clock ticks
  // The rise time of SCL and SDA can be up to 1000nS (in standard mode)
  // so it is essential to debounce the inputs.
  // The spec requires 0.05V of hysteresis, but in practise
  // simply debouncing the inputs is sufficient
  // I2C spec requires suppresion of spikes of 
  // maximum duration 50nS, so this debounce time should be greater than 50nS
  // Also increases data hold time and decreases data setup time
  // during an I2C read operation
  // 10 ticks = 208nS @ 48MHz
  `define DEB_I2C_LEN (10*`CLK_FREQ)/48

  // Delay SCL for use as internal sampling clock
  // Using delayed version of SCL to ensure that 
  // SDA is stable when it is sampled.
  // Not entirely citical, as according to I2C spec
  // SDA should have a minimum of 100nS of set up time
  // with respect to SCL rising edge. But with the very slow edge 
  // speeds used in I2C it is better to err on the side of caution.
  // This delay also has the effect of adding extra hold time to the data
  // with respect to SCL falling edge. I2C spec requires 0nS of data hold time.
  // 10 ticks = 208nS @ 48MHz
  `define SCL_DEL_LEN (10*`CLK_FREQ)/48

  // Delay SDA for use in start/stop detection
  // Use delayed SDA during start/stop detection to avoid
  // incorrect detection at SCL falling edge.
  // From I2C spec start/stop setup is 600nS with respect to SCL rising edge
  // and start/stop hold is 600nS wrt SCL falling edge.
  // So it is relatively easy to discriminate start/stop,
  // but data setup time is a minimum of 100nS with respect to SCL rising edge
  // and 0nS hold wrt to SCL falling edge.
  // So the tricky part is providing robust start/stop detection
  // in the presence of regular data transitions.
  // This delay time should be less than 100nS
  // 4 ticks = 83nS @ 48MHz
  `define SDA_DEL_LEN (4*`CLK_FREQ)/48
  module registerInterface (
    clk,
    addr,
    dataIn,
    writeEn,
    dataOut,
    myReg0,
    myReg1,
    myReg2,
    myReg3,
    myReg4,
    myReg5,
    myReg6,
    myReg7

  );
  input clk;
  input [7:0] addr;
  input [7:0] dataIn;
  input writeEn;
  output [7:0] dataOut;
  output [7:0] myReg0;
  output [7:0] myReg1;
  output [7:0] myReg2;
  output [7:0] myReg3;
  input [7:0] myReg4;
  input [7:0] myReg5;
  input [7:0] myReg6;
  input [7:0] myReg7;

  reg [7:0] dataOut;
  reg [7:0] myReg0;
  reg [7:0] myReg1;
  reg [7:0] myReg2;
  reg [7:0] myReg3;

  // --- I2C Read
  always @(posedge clk) begin
    case (addr)
      8'h00: dataOut <= myReg0;  
      8'h01: dataOut <= myReg1;  
      8'h02: dataOut <= myReg2;  
      8'h03: dataOut <= myReg3;  
      8'h04: dataOut <= myReg4;  
      8'h05: dataOut <= myReg5;  
      8'h06: dataOut <= myReg6;  
      8'h07: dataOut <= myReg7;  
      default: dataOut <= 8'h00;
    endcase
  end

  // --- I2C Write
  always @(posedge clk) begin
    if (writeEn == 1'b1) begin
      case (addr)
        8'h00: myReg0 <= dataIn;  
        8'h01: myReg1 <= dataIn;
        8'h02: myReg2 <= dataIn;
        8'h03: myReg3 <= dataIn;
      endcase
    end
  end

  endmodule

  // ----------------------- i2cSlave_define.v --------------------

  // stream states
  `define STREAM_IDLE 2'b00
  `define STREAM_READ 2'b01
  `define STREAM_WRITE_ADDR 2'b10
  `define STREAM_WRITE_DATA 2'b11

  // start stop detection states
  `define NULL_DET 2'b00
  `define START_DET 2'b01
  `define STOP_DET 2'b10

  // i2c ack and nak
  `define I2C_NAK 1'b1
  `define I2C_ACK 1'b0

  // ----------------------------------------------------------------
  // ------------- modify constants below this line -----------------
  // ----------------------------------------------------------------

  // i2c device address
  `define I2C_ADDRESS 7'h5C

  // System clock frequency in MHz
  // If you are using a clock frequency below 24MHz, then the macro
  // for SDA_DEL_LEN will result in compile errors for i2cSlave.v
  // you will need to hand tweak the SDA_DEL_LEN constant definition
  `define CLK_FREQ 24

  // Debounce SCL and SDA over this many clock ticks
  // The rise time of SCL and SDA can be up to 1000nS (in standard mode)
  // so it is essential to debounce the inputs.
  // The spec requires 0.05V of hysteresis, but in practise
  // simply debouncing the inputs is sufficient
  // I2C spec requires suppresion of spikes of 
  // maximum duration 50nS, so this debounce time should be greater than 50nS
  // Also increases data hold time and decreases data setup time
  // during an I2C read operation
  // 10 ticks = 208nS @ 48MHz
  `define DEB_I2C_LEN (10*`CLK_FREQ)/48

  // Delay SCL for use as internal sampling clock
  // Using delayed version of SCL to ensure that 
  // SDA is stable when it is sampled.
  // Not entirely citical, as according to I2C spec
  // SDA should have a minimum of 100nS of set up time
  // with respect to SCL rising edge. But with the very slow edge 
  // speeds used in I2C it is better to err on the side of caution.
  // This delay also has the effect of adding extra hold time to the data
  // with respect to SCL falling edge. I2C spec requires 0nS of data hold time.
  // 10 ticks = 208nS @ 48MHz
  `define SCL_DEL_LEN (10*`CLK_FREQ)/48

  // Delay SDA for use in start/stop detection
  // Use delayed SDA during start/stop detection to avoid
  // incorrect detection at SCL falling edge.
  // From I2C spec start/stop setup is 600nS with respect to SCL rising edge
  // and start/stop hold is 600nS wrt SCL falling edge.
  // So it is relatively easy to discriminate start/stop,
  // but data setup time is a minimum of 100nS with respect to SCL rising edge
  // and 0nS hold wrt to SCL falling edge.
  // So the tricky part is providing robust start/stop detection
  // in the presence of regular data transitions.
  // This delay time should be less than 100nS
  // 4 ticks = 83nS @ 48MHz
  `define SDA_DEL_LEN (4*`CLK_FREQ)/48

  module serialInterface (clearStartStopDet, clk, dataIn, dataOut, regAddr, rst, scl, sdaIn, sdaOut, sdaEn,  startStopDetState, writeEn);
  input   clk;
  input   [7:0]dataIn;
  input   rst;
  input   scl;
  input   sdaIn;
  input   [1:0]startStopDetState;
  output  clearStartStopDet;
  output  [7:0]dataOut;
  output  [7:0]regAddr;
  output  sdaOut;
  output	sdaEn;
  output  writeEn;

  reg     clearStartStopDet, next_clearStartStopDet;
  wire    clk;
  wire    [7:0]dataIn;
  reg     [7:0]dataOut, next_dataOut;
  reg     [7:0]regAddr, next_regAddr;
  wire    rst;
  wire    scl;
  wire    sdaIn;
  reg     sdaOut, next_sdaOut;
  reg		sdaEn, next_sdaEn;
  wire    [1:0]startStopDetState;
  reg     writeEn, next_writeEn;

  // diagram signals declarations
  reg  [2:0]bitCnt, next_bitCnt;
  reg  [7:0]rxData, next_rxData;
  reg  [1:0]streamSt, next_streamSt;
  reg  [7:0]txData, next_txData;


  // BINARY ENCODED state machine: SISt
  // State codes definitions:
  `define START 					4'b0000
  `define CHK_RD_WR 				4'b0001
  `define READ_RD_LOOP 			4'b0010
  `define READ_WT_HI 				4'b0011
  `define READ_CHK_LOOP_FIN 		4'b0100
  `define READ_WT_LO 				4'b0101
  `define READ_WT_ACK 			4'b0110
  `define WRITE_WT_LO 			4'b0111
  `define WRITE_WT_HI 			4'b1000
  `define WRITE_CHK_LOOP_FIN 		4'b1001
  `define WRITE_LOOP_WT_LO 		4'b1010
  `define WRITE_ST_LOOP 			4'b1011
  `define WRITE_WT_LO2 			4'b1100
  `define WRITE_WT_HI2 			4'b1101
  `define WRITE_CLR_WR 			4'b1110
  `define WRITE_CLR_ST_STOP 		4'b1111

  reg [3:0]CurrState_SISt, NextState_SISt;

  // Diagram actions (continuous assignments allowed only: assign ...)
  // diagram ACTION


  // Machine: SISt

  // NextState logic (combinatorial)
  always @ (startStopDetState or streamSt or scl or txData or bitCnt or rxData or sdaIn or regAddr or dataIn or sdaOut or writeEn or dataOut or clearStartStopDet or CurrState_SISt)
  begin
    NextState_SISt <= CurrState_SISt;
    // Set default values for outputs and signals
    next_streamSt <= streamSt;
    next_txData <= txData;
    next_rxData <= rxData;
    next_sdaOut <= sdaOut;
    next_sdaEn <= sdaEn;
    next_writeEn <= writeEn;
    next_dataOut <= dataOut;
    next_bitCnt <= bitCnt;
    next_clearStartStopDet <= clearStartStopDet;
    next_regAddr <= regAddr;
    case (CurrState_SISt)  // synopsys parallel_case full_case
      `START:
      begin
        next_streamSt <= `STREAM_IDLE;
        next_txData <= 8'h00;
        next_rxData <= 8'h00;
        next_sdaOut <= 1'b1;
        next_sdaEn <= 1'b0;
      next_writeEn <= 1'b0;
        next_dataOut <= 8'h00;
        next_bitCnt <= 3'b000;
        next_clearStartStopDet <= 1'b0;
        NextState_SISt <= `CHK_RD_WR;
      end
      `CHK_RD_WR:
      begin
        if (streamSt == `STREAM_READ)
        begin
          NextState_SISt <= `READ_RD_LOOP;
          next_txData <= dataIn;
          next_regAddr <= regAddr + 1'b1;
          next_bitCnt <= 3'b001;
        end
        else
        begin
          NextState_SISt <= `WRITE_WT_HI;
          next_rxData <= 8'h00;
        end
      end
      `READ_RD_LOOP:
      begin
        if (scl == 1'b0)
        begin
          NextState_SISt <= `READ_WT_HI;
          next_sdaOut <= txData [7];
      next_sdaEn <= 1'b1;
          next_txData <= {txData [6:0], 1'b0};
        end
      end
      `READ_WT_HI:
      begin
        if (scl == 1'b1)
        begin
          NextState_SISt <= `READ_CHK_LOOP_FIN;
        end
      end
      `READ_CHK_LOOP_FIN:
      begin
        if (bitCnt == 3'b000)
        begin
          NextState_SISt <= `READ_WT_LO;
        end
        else
        begin
          NextState_SISt <= `READ_RD_LOOP;
          next_bitCnt <= bitCnt + 1'b1;
        end
      end
      `READ_WT_LO:
      begin
        if (scl == 1'b0)
        begin
          NextState_SISt <= `READ_WT_ACK;
          next_sdaOut <= 1'b1;
      next_sdaEn <= 1'b0;
        end
      end
      `READ_WT_ACK:
      begin
        if (scl == 1'b1)
        begin
          NextState_SISt <= `CHK_RD_WR;
          if (sdaIn == `I2C_NAK)
          next_streamSt <= `STREAM_IDLE;
        end
      end
      `WRITE_WT_LO:
      begin
        if ((scl == 1'b0) && (startStopDetState == `STOP_DET || 
          (streamSt == `STREAM_IDLE && startStopDetState == `NULL_DET)))
        begin
          NextState_SISt <= `WRITE_CLR_ST_STOP;
          case (startStopDetState)
          `NULL_DET:       		next_bitCnt <= bitCnt + 1'b1;
          `START_DET: begin        next_streamSt <= `STREAM_IDLE;        next_rxData <= 8'h00;        end
          default: ;
          endcase
          next_streamSt <= `STREAM_IDLE;
          next_clearStartStopDet <= 1'b1;
        end
        else if (scl == 1'b0)
        begin
          NextState_SISt <= `WRITE_ST_LOOP;
          case (startStopDetState)
          `NULL_DET:	       next_bitCnt <= bitCnt + 1'b1;
          `START_DET: begin  next_streamSt <= `STREAM_IDLE; next_rxData <= 8'h00; end
          default: ;
          endcase
        end
      end
      `WRITE_WT_HI:
      begin
        if (scl == 1'b1)
        begin
          NextState_SISt <= `WRITE_WT_LO;
          next_rxData <= {rxData [6:0], sdaIn};
          next_bitCnt <= 3'b000;
        end
      end
      `WRITE_CHK_LOOP_FIN:
      begin
        if (bitCnt == 3'b111)
        begin
          NextState_SISt <= `WRITE_CLR_WR;
          next_sdaOut <= `I2C_ACK;
          next_sdaEn <= 1'b1;
          case (streamSt)
          `STREAM_IDLE: 
        begin
            if (rxData[7:1] == `I2C_ADDRESS && startStopDetState == `START_DET) 
        begin
              if (rxData[0] == 1'b1)	next_streamSt <= `STREAM_READ;
          else			        next_streamSt <= `STREAM_WRITE_ADDR;
          end
            else
              next_sdaOut <= `I2C_NAK;
        next_sdaEn <= 1'b1;
            end
          `STREAM_WRITE_ADDR: 
        begin
            next_streamSt <= `STREAM_WRITE_DATA;
            next_regAddr <= rxData;
            end
          `STREAM_WRITE_DATA:
        begin
          next_dataOut <= rxData;
            next_writeEn <= 1'b1;
            end
            default: next_streamSt <= streamSt;
          endcase
        end
        else
        begin
          NextState_SISt <= `WRITE_ST_LOOP;
          next_bitCnt <= bitCnt + 1'b1;
        end
      end
      `WRITE_LOOP_WT_LO:
      begin
        if (scl == 1'b0)
        begin
          NextState_SISt <= `WRITE_CHK_LOOP_FIN;
        end
      end
      `WRITE_ST_LOOP:
      begin
        if (scl == 1'b1)
        begin
          NextState_SISt <= `WRITE_LOOP_WT_LO;
          next_rxData <= {rxData [6:0], sdaIn};
        end
      end
      `WRITE_WT_LO2:
      begin
        if (scl == 1'b0)
        begin
          NextState_SISt <= `CHK_RD_WR;
          next_sdaOut <= 1'b1;
      next_sdaEn <= 1'b0;
        end
      end
      `WRITE_WT_HI2:
      begin
        next_clearStartStopDet <= 1'b0;
        if (scl == 1'b1)
        begin
          NextState_SISt <= `WRITE_WT_LO2;
        end
      end
      `WRITE_CLR_WR:
      begin
        if (writeEn == 1'b1)
        next_regAddr <= regAddr + 1'b1;
        next_writeEn <= 1'b0;
        next_clearStartStopDet <= 1'b1;
        NextState_SISt <= `WRITE_WT_HI2;
      end
      `WRITE_CLR_ST_STOP:
      begin
        next_clearStartStopDet <= 1'b0;
        NextState_SISt <= `CHK_RD_WR;
      end
    endcase
  end

  // Current State Logic (sequential)
  always @ (posedge clk)
  begin
    if (rst == 1'b1)
      CurrState_SISt <= `START;
    else
      CurrState_SISt <= NextState_SISt;
  end

  // Registered outputs logic
  always @ (posedge clk)
  begin
    if (rst == 1'b1)
    begin
      sdaOut <= 1'b1;
      sdaEn <= 1'b0;
      writeEn <= 1'b0;
      dataOut <= 8'h00;
      clearStartStopDet <= 1'b0;
      regAddr <=  8'h00;    // Initialization in the reset state or default value required!!
      streamSt <= `STREAM_IDLE;
      txData <= 8'h00;
      rxData <= 8'h00;
      bitCnt <= 3'b000;
    end
    else 
    begin
      sdaOut <= next_sdaOut;
      sdaEn <= next_sdaEn;
      writeEn <= next_writeEn;
      dataOut <= next_dataOut;
      clearStartStopDet <= next_clearStartStopDet;
      regAddr <= next_regAddr;
      streamSt <= next_streamSt;
      txData <= next_txData;
      rxData <= next_rxData;
      bitCnt <= next_bitCnt;
    end
  end

  endmodule

`timescale 1ns/10ps

module tb_i2c();

  reg [2:0]REG;
  reg CLK, RESET, rESET;

  reg [7:0] DATA_I;
  reg [7:0] TXR;

  tri1 I2c_scl, I2c_sda;
  wire sda_pad_i, sda_pad_o;

  wire [7:0] DATA_O;

  wire SDA_pad;
  wire [7:0] myreg0;

  localparam PER = 20.0;
  parameter  HPER = (PER) / 2.0; 

  assign I2c_sda    = (SDA_pad == 1) ? sda_pad_o : 1'bz;
  assign sda_pad_i  = (SDA_pad == 0) ? I2c_sda : 1'bz;
  
  i2c_top tb(
    .clk       (CLK       ),
    .reset     (RESET     ),
    .data_i    (DATA_I    ),
    .txr       (TXR       ),
    .reg_addr  (REG       ),
    .sda_pad_i (sda_pad_i ),
    .sda_pad_o (sda_pad_o ), 
    .sda_pad   (SDA_pad   ), //tri state enable
    .SCL       (I2c_scl   )
  );

  i2cSlaveTop s (
		.CLK    (CLK      ),
		.CLR    (rESET    ),
		.scl    (I2c_scl  ), 
		.sda    (I2c_sda  ), 
		.myReg0 (myreg0   )
  );

  initial begin
    CLK = 1'b0;
    RESET = 1'b1;
    rESET = 1'b1;
    REG = 1'b0;
  end

  always #(HPER) CLK <= ~CLK;

  initial begin
    #(PER*2) 
    RESET = 1'b0;
    rESET = 1'b0;
  end
 
  initial begin
    /////////////////////////
    ///////data write////////
    /////////////////////////
    // prer
    #(PER*2)
    REG = 3'b001;
    DATA_I = 8'b0000_0101 ;
    // prer
    #(PER)
    REG = 3'b010;  
    DATA_I = 8'b0000_0000;
    // cr
    #(PER)
    REG = 3'b011;
    DATA_I = 8'b1010_0000;
    //addr + write
    #(PER)
    REG = 3'b100;
    DATA_I = 8'b1011_1000;
    TXR    = 8'b0000_0000;

    // enable
    #(PER)
    REG = 3'b101;

    #(PER*260)
    REG = 3'b110;
    TXR = 8'b0101_0010;

    #(PER*90)
    REG = 3'b101;

    #(PER*220)
    REG = 3'b110; 

    #(PER)
    REG = 3'b011;
    DATA_I = 8'b0100_0000;   

    #(PER*90)
    REG = 3'b101; 
    /////////////////////////
    ///////data read////////
    /////////////////////////
    // reset
    #(PER*300)
    RESET = 1'b1;
    #(PER*50)
    RESET = 1'b0;   
    
    /////////////// step-1 /////////////////  
    
    // prer
    #(PER*5)
    REG = 3'b001;
    DATA_I = 8'b0000_0101; 
    // prer
    #(PER)
    REG = 3'b010;  
    DATA_I = 8'b0000_0000;
    // cr
    #(PER)
    REG = 3'b011;
    DATA_I = 8'b1010_0000;
    //addr + read
    #(PER)
    REG = 3'b100;
    DATA_I = 8'b1011_1000;

    // enable
    #(PER)
    REG = 3'b101;
    TXR = 8'b0000_0000;    
    #(PER*260)
    REG = 3'b110;
    
    #(PER*90)
    REG = 3'b101;
    #(PER*220)
    REG = 3'b110;

    /////////////// step-2 /////////////////  
    #(PER*50)
    RESET = 1'b1;
    #(PER*50)
    RESET = 1'b0;  

     // prer
    #(PER*5)
    REG = 3'b001;
    DATA_I = 8'b0000_0101; 
    // prer
    #(PER)
    REG = 3'b010;  
    DATA_I = 8'b0000_0000;
    // cr
    #(PER)
    REG = 3'b011;
    DATA_I = 8'b1001_0000;
    //addr + read
    #(PER)
    REG = 3'b100;
    DATA_I = 8'b1011_1001;

    #(PER)
    REG = 3'b101;  
    #(PER*260)
    REG = 3'b110;    

    #(PER*90)
    REG = 3'b101;
    #(PER*10)
    REG = 3'b011;
    DATA_I = 8'b0100_1000;          
    // #(PER*220)
    // REG = 3'b110; 
  end

  initial begin
    $dumpfile("I2C.vcd");
    $dumpvars(0, tb_i2c);
    #(PER*5000) $finish; 
  end

endmodule // tb_i2c