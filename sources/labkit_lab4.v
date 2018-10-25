`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Updated 9/29/2017 V2.0
// Create Date: 10/1/2015 V1.0
// Design Name: 
// Module Name: labkit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module labkit(
   input CLK100MHZ,
   input[15:0] SW, 
   input BTNC, BTNU, BTNL, BTNR, BTND,
   output[3:0] VGA_R, 
   output[3:0] VGA_B, 
   output[3:0] VGA_G,
   output[7:0] JA, 
   output VGA_HS, 
   output VGA_VS, 
   output LED16_B, LED16_G, LED16_R,
   output LED17_B, LED17_G, LED17_R,
   output[15:0] LED,
   output[7:0] SEG,  // segments A-G (0-6), DP (7)
   output[7:0] AN    // Display 0-7
   );
   

// create 25mhz system clock
    wire clock_25mhz;
    clock_quarter_divider clockgen(.clk100_mhz(CLK100MHZ), .clock_25mhz(clock_25mhz));

//  instantiate 7-segment display;  
    wire [31:0] data;
    wire [6:0] segments;
    display_8hex display(.clk(clock_25mhz),.data(data), .seg(segments), .strobe(AN));    
    assign SEG[6:0] = segments;
    assign SEG[7] = 1'b1;

//////////////////////////////////////////////////////////////////////////////////
//

//declare wires as shown in Figure 2 to connect the submodules:
wire reset_sync;    
wire one_hz_enable;
wire [3:0] value;
wire expired, start_timer; 
wire [1:0] status_indicator;
wire hidden_switch, brake_depressed_switch, driver_door_switch, passenger_door_switch, ignition_switch, reprogram;
wire [1:0] time_parameter_selector;
wire [3:0] time_value;
wire fuel_pump_power;
wire [1:0] interval;
wire [15:0] parameters; //keeps track of all time parameter values
wire alarm, sound;
wire LED_0_input;
wire [3:0] output_hex;
wire [2:0] fsm_state;    
//instantiate the submodules and wire their inputs and outputs
//(use the labkit's clock_25mhz as the clock to all blocks)	
synchronize sync (.clk(clock_25mhz), .in(SW[15]), .out(reset_sync));

divider divider1 (.clock_25mhz(clock_25mhz), .reset_sync(reset_sync||start_timer),
     .one_hz_enable(one_hz_enable));
timer timer1(.value(value), .start_timer(start_timer), .clock_25mhz(clock_25mhz),
      .one_hz_enable(one_hz_enable), .reset_sync(reset_sync), .expired(expired), .output_hex(output_hex));
     


debounce brake_depressed_switch_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(BTND), 
	         .clean(brake_depressed_switch));
debounce hidden_switch_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(BTNU), 
             .clean(hidden_switch));
debounce driver_door_switch_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(BTNL), 
             .clean(driver_door_switch));
debounce passenger_door_switch_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(BTNR), 
             .clean(passenger_door_switch));                                                                 
debounce ignition_switch_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[7]), 
             .clean(ignition_switch));  
debounce reprogram_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(BTNC), 
             .clean(reprogram));               
debounce time_parameter_selector_0_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[4]), 
             .clean(time_parameter_selector[0]));    
debounce time_parameter_selector_1_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[5]), 
             .clean(time_parameter_selector[1]));     
debounce time_value_0_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[0]), 
             .clean(time_value[0])); 
debounce time_value_1_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[1]), 
             .clean(time_value[1])); 
debounce time_value_2_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[2]), 
             .clean(time_value[2]));        
debounce time_value_3_debounce(.reset(reset_sync), .clock(clock_25mhz), .noisy(SW[3]), 
             .clean(time_value[3]));     

fuel_pump_fsm fpFSM(.hidden_switch(hidden_switch), .brake_depressed_switch(brake_depressed_switch),
             .ignition_switch(ignition_switch), .clock_25mhz(clock_25mhz), .reset_sync(reset_sync),
             .fuel_pump_power(fuel_pump_power));
time_parameters tp(.time_parameter_selector(time_parameter_selector), .time_value(time_value),
             .interval(interval), .clk(clock_25mhz), .reprogram(reprogram), .reset_sync(reset_sync), .value(value),
             .parameters(parameters));              
anti_theft_fsm atFSM(.expired(expired), .ignition_switch(ignition_switch), .driver_door(driver_door_switch),
             .passenger_door(passenger_door_switch), .reprogram(reprogram), .clock_25mhz(clock_25mhz),
             .reset_sync(reset_sync), .interval(interval), .start_timer(start_timer), 
             .status_indicator(status_indicator), .alarm(alarm), .fsm_state(fsm_state));                   

LED_0_status status_indictor_implement(.status_indicator(status_indicator), .clock_25mhz(clock_25mhz),
            .one_hz_enable(one_hz_enable), .LED_0_input(LED_0_input)); 
           
siren_generator(.alarm(alarm), .clock_25mhz(clock_25mhz), .sound(sound));
                                      
assign data = {fsm_state, output_hex}; //{{0, fsm_state}, {0, fuel_fsm_state}, {2'b00, status_indicator}, {3'b000, sound}, {3'b000, alarm},
//8'h56, output_hex};
assign LED[0] = LED_0_input;
assign LED[1] = fuel_pump_power;    
assign JA[0] = sound;                                                                              
//////////////////////////////////////////////////////////////////////////////////
// sample Verilog to generate color bars
    
    wire [9:0] hcount;
    wire [9:0] vcount;
    wire hsync, vsync, at_display_area;
    vga vga1(.vga_clock(clock_25mhz),.hcount(hcount),.vcount(vcount),
          .hsync(hsync),.vsync(vsync),.at_display_area(at_display_area));
        
    assign VGA_R = at_display_area ? {4{hcount[7]}} : 0;
    assign VGA_G = at_display_area ? {4{hcount[6]}} : 0;
    assign VGA_B = at_display_area ? {4{hcount[5]}} : 0;
    assign VGA_HS = ~hsync;
    assign VGA_VS = ~vsync;
endmodule

module clock_quarter_divider(input clk100_mhz, output reg clock_25mhz = 0);
    reg counter = 0;

    // VERY BAD VERILOG
    // VERY BAD VERILOG
    // VERY BAD VERILOG
    // But it's a quick and dirty way to create a 25Mhz clock
    // Please use the IP Clock Wizard under FPGA Features/Clocking
    //
    // For 1 Hz pulse, it's okay to use a counter to create the pulse as in
    // assign onehz = (counter == 100_000_000); 
    // be sure to have the right number of bits.

    always @(posedge clk100_mhz) begin
        counter <= counter + 1;
        if (counter == 0) begin
            clock_25mhz <= ~clock_25mhz;
        end
    end
endmodule

module vga(input vga_clock,
            output reg [9:0] hcount = 0,    // pixel number on current line
            output reg [9:0] vcount = 0,    // line number
            output reg vsync, hsync, 
            output at_display_area);

   // Comments applies to XVGA 1024x768, left in for reference
   // horizontal: 1344 pixels total
   // display 1024 pixels per line
   reg hblank,vblank;
   wire hsyncon,hsyncoff,hreset,hblankon;
   assign hblankon = (hcount == 639);    // active H  1023
   assign hsyncon = (hcount == 655);     // active H + FP 1047
   assign hsyncoff = (hcount == 751);    // active H + fp + sync  1183
   assign hreset = (hcount == 799);      // active H + fp + sync + bp 1343

   // vertical: 806 lines total
   // display 768 lines
   wire vsyncon,vsyncoff,vreset,vblankon;
   assign vblankon = hreset & (vcount == 479);    // active V   767
   assign vsyncon = hreset & (vcount ==490 );     // active V + fp   776
   assign vsyncoff = hreset & (vcount == 492);    // active V + fp + sync  783
   assign vreset = hreset & (vcount == 523);      // active V + fp + sync + bp 805

   // sync and blanking
   wire next_hblank,next_vblank;
   assign next_hblank = hreset ? 0 : hblankon ? 1 : hblank;
   assign next_vblank = vreset ? 0 : vblankon ? 1 : vblank;
   always @(posedge vga_clock) begin
      hcount <= hreset ? 0 : hcount + 1;
      hblank <= next_hblank;
      hsync <= hsyncon ? 0 : hsyncoff ? 1 : hsync;  // active low

      vcount <= hreset ? (vreset ? 0 : vcount + 1) : vcount;
      vblank <= next_vblank;
      vsync <= vsyncon ? 0 : vsyncoff ? 1 : vsync;  // active low

   end

   assign at_display_area = ((hcount >= 0) && (hcount < 640) && (vcount >= 0) && (vcount < 480));

endmodule

module anti_theft_fsm(input expired, 
                    input ignition_switch,
                    input driver_door,
                    input passenger_door,
                    input reprogram,
                    input clock_25mhz,
                    input reset_sync,
                    output reg [1:0] interval,
                    output reg start_timer,
                    output reg [1:0] status_indicator,
                    output reg alarm,
                    output [2:0] fsm_state);
    parameter ARMED = 0;
    parameter TRIGGERED = 1;
    parameter SOUND_ALARM = 2;
    parameter EXPIRING_ALARM = 3;
    parameter DISARMED = 4;
    parameter DISARMED_1 = 5;
    parameter DISARMED_2 = 6;
    parameter DISARMED_3 = 7;
    
    parameter LIGHT_OFF = 0;
    parameter BLINKING = 1;
    parameter CONSTANTLY_ON = 2;
    
    parameter T_ARM_DELAY = 0;
    parameter T_DRIVER_DELAY = 1;
    parameter T_PASSENGER_DELAY = 2;
    parameter T_ALARM_ON = 3;
    
    
    reg [2:0] state;
    
    always @(posedge clock_25mhz)
        begin
            if (reprogram || reset_sync) //reset to ARMED state
                begin
                    state <= ARMED;
                    start_timer <= 0;
                    alarm <= 0;
                    status_indicator <= BLINKING; 
                end
            case (state)
                ARMED:
                    begin
                        status_indicator <= BLINKING; 
                        alarm <= 0;
                        
                        if (ignition_switch)
                            begin
                                state <= DISARMED;
                            end
                        else if (driver_door)
                            begin
                                state <= TRIGGERED;
                                start_timer <= 1;
                            end
                        else if (passenger_door)
                            begin
                                state <= TRIGGERED;
                                start_timer <= 1;
                            end
                    end
                    
                TRIGGERED: 
                    begin
                        status_indicator <= CONSTANTLY_ON;
                        start_timer <= 0;
                        alarm <= 0;
                        if (ignition_switch)
                            begin
                                state <= DISARMED;
                            end
                        else if (expired)
                            begin
                                state <= SOUND_ALARM;
                            end
                    end
                
                SOUND_ALARM:
                    begin
                        status_indicator <= CONSTANTLY_ON;
                        alarm <= 1;
                        start_timer <= 0;
                        if (ignition_switch)
                            begin
                                state <= DISARMED;
                            end
                        else if (~passenger_door && ~driver_door) //if both not open
                            begin
                                start_timer <= 1;
                                state <= EXPIRING_ALARM;
                            end
                    end
                EXPIRING_ALARM:
                    begin
                        status_indicator <= CONSTANTLY_ON;
                        alarm <= 1;
                        start_timer <= 0;
                        if (ignition_switch)
                            begin
                                state <= DISARMED;
                            end
                        else if (expired)
                            begin
                                state <= ARMED;
                            end
                        else if (passenger_door || driver_door) //if one of them open
                            begin
                                state <= SOUND_ALARM;
                            end
                    end
                DISARMED:
                    begin
                        status_indicator <= LIGHT_OFF;
                        alarm <= 0;
                        if (~ignition_switch)
                            begin
                                state <= DISARMED_1;
                            end
                    end
                DISARMED_1:
                    begin
                        status_indicator <= LIGHT_OFF;
                        alarm <= 0;
                        if (ignition_switch)
                            begin
                                state <= DISARMED;
                            end
                        else if (driver_door)
                            begin
                                state <= DISARMED_2;
                            end
                    end
                DISARMED_2:
                    begin
                        status_indicator <= LIGHT_OFF;
                        alarm <= 0;
                        if (ignition_switch)
                            begin
                                state <= DISARMED;
                            end
                        else if (~driver_door)
                            begin
                                state <= DISARMED_3;
                                start_timer <= 1;
                            end
                    end
                 
                 DISARMED_3:
                    begin
                        status_indicator <= LIGHT_OFF;
                        alarm <= 0;
                        start_timer <= 0;
                        if (driver_door || passenger_door)
                            begin
                                state <= DISARMED;
                            end
                        else if (expired)
                            begin
                                state <= ARMED;
                            end
                    end
                 default: //default is ARMED state
                    begin
                        start_timer <= 0;
                        status_indicator <= BLINKING;
                        alarm <= 0;
                        state <= ARMED;
                    end
                
            endcase
        end
        
        always @(*) //immediately set interval to correct time value
            begin
                case (state)
                    ARMED:
                        begin
                            if (driver_door)
                                begin
                                    interval <= T_DRIVER_DELAY;
                                end
                            else if (passenger_door)
                                begin
                                    interval <= T_PASSENGER_DELAY;
                                end
                        end
                    
                    SOUND_ALARM:
                        begin
                            if (~passenger_door && ~driver_door)
                                begin
                                     interval <= T_ALARM_ON;
                                end
                        end
                    DISARMED_2:
                        begin
                            if (~driver_door)
                                begin
                                    interval <= T_ARM_DELAY;
                                end
                        end
                            
                endcase
            end 
            
        assign fsm_state = state;                    
endmodule

// Switch Debounce Module
// use your system clock for the clock input
// to produce a synchronous, debounced output
module debounce #(parameter DELAY=1000000)   // .01 sec with a 100Mhz clock
	        (input reset, clock, noisy,
	         output reg clean);

   reg [19:0] count;
   reg new;

   always @(posedge clock)
     if (reset)
       begin
	  count <= 0;
	  new <= noisy;
	  clean <= noisy;
       end
     else if (noisy != new)
       begin
	  new <= noisy;
	  count <= 0;
       end
     else if (count == DELAY)
       clean <= new;
     else
       count <= count+1;
      
endmodule

// pulse synchronizer
module synchronize #(parameter NSYNC = 2)  // number of sync flops.  must be >= 2
                   (input clk,in,
                    output reg out);

  reg [NSYNC-2:0] sync;

  always @ (posedge clk)
  begin
    {out,sync} <= {sync[NSYNC-2:0],in};
  end
endmodule

module time_parameters(input [1:0] time_parameter_selector,
            input [3:0] time_value,
            input [1:0] interval,
            input clk,
            input reprogram, 
            input reset_sync,
            output reg [3:0] value,
            output reg [15:0] parameters);
    
    parameter T_ARM_DELAY = 0;
    parameter T_DRIVER_DELAY = 1;
    parameter T_PASSENGER_DELAY = 2;
    parameter T_ALARM_ON = 3;
    
    initial 
        begin
            parameters = 16'b0110100011111010; //{T_ARM_DELAY, T_DRIVER_DELAY, T_PASSENGER_DELAY, T_ALARM_ON}
        end
    
    
    always @(posedge clk) 
        begin
            if (reset_sync)
                begin
                    parameters <= 16'b0110100011111010;
                end
            else if (reprogram)
                begin
                    case (time_parameter_selector)
                        T_ARM_DELAY:
                            begin
                                parameters[15:12] <= time_value;
                            end
                        T_DRIVER_DELAY:
                            begin
                                parameters[11:8] <= time_value;
                            end
                        T_PASSENGER_DELAY:
                            begin
                                parameters[7:4] <= time_value;
                            end
                        T_ALARM_ON:
                            begin
                                parameters[3:0] <= time_value;
                            end
                    endcase
                end
            else
                begin
                    case (interval)
                        T_ARM_DELAY:
                            begin
                                value <= parameters[15:12];
                            end
                        T_DRIVER_DELAY:
                            begin
                                value <= parameters[11:8];
                            end
                        T_PASSENGER_DELAY:
                            begin
                                value <= parameters[7:4];
                            end
                        T_ALARM_ON:
                            begin
                                value <= parameters[3:0];
                            end
                    endcase
                end
        end            

endmodule

module timer(input [3:0] value, 
          input start_timer, 
          input clock_25mhz, 
          input one_hz_enable,
          input reset_sync,
          output reg expired,
          output [3:0] output_hex);
    
    reg [3:0] internal_counter; 
    
    initial begin
        expired = 0;
        internal_counter = 0;  
    end
    
    always @(posedge clock_25mhz)  
        begin
            expired <= 0;
            if (reset_sync)
                begin
                    internal_counter <= 0;  
                end
            else if (start_timer)
                begin
                    internal_counter <= value;  
                end
            if (internal_counter == 0)
                begin
                    expired <= 1;
                end
            else if (one_hz_enable && !expired) //every sec, decrement counter by 1
                begin
                    internal_counter <= internal_counter - 1;    
                end
                
        end  
    assign output_hex = internal_counter;
    
endmodule

module divider (input clock_25mhz, 
            input reset_sync,
            output reg one_hz_enable);
    
    reg [24:0] counter; 
    
    always @(posedge clock_25mhz)    
        begin  
            if (reset_sync)
                begin
                    counter <= 0;
                    one_hz_enable <= 0;
                end
            else if (counter == 25000000)
                begin
                    counter <= 0;
                    one_hz_enable <= 1;
                end
            else
                begin
                    counter <= counter + 1;
                    one_hz_enable <= 0;
                end
        end          
    
endmodule

module fuel_pump_fsm(input hidden_switch, 
                  input brake_depressed_switch,
                  input ignition_switch,
                  input clock_25mhz,
                  input reset_sync,
                  output reg fuel_pump_power);
      
    parameter FUEL_PUMP_OFF = 0;
    parameter POWER_RESTORE_START = 1;
    parameter POWER_RESTORE = 2;
    parameter POWER_LATCH_ON = 3;
    
    reg [2:0] state;
      
    always @(posedge clock_25mhz)
        begin
            if (reset_sync)
                begin
                    state <= FUEL_PUMP_OFF;
                    fuel_pump_power <= 0;
                end
            case (state)
                FUEL_PUMP_OFF:
                    begin
                        fuel_pump_power <= 0;
                        if (ignition_switch)
                            begin
                                state <= POWER_RESTORE_START;
                            end
                    end
                
                POWER_RESTORE_START:
                    begin
                        if (hidden_switch && brake_depressed_switch)
                            begin
                                state <= POWER_RESTORE;
                            end
                    end
                POWER_RESTORE:
                    begin
                        fuel_pump_power <= 1;
                        state <= POWER_LATCH_ON;
                    end
                    
                POWER_LATCH_ON:
                    begin
                        if (!ignition_switch)
                            begin
                                state <= FUEL_PUMP_OFF;
                            end
                    end
                default:
                    begin
                        state <= FUEL_PUMP_OFF;
                    end
              
            endcase
        end
        
endmodule

module LED_0_status(input [1:0] status_indicator, 
                input clock_25mhz,
                input one_hz_enable, 
                output reg LED_0_input);   
                 
    parameter LIGHT_OFF = 0;
    parameter BLINKING = 1;
    parameter CONSTANTLY_ON = 2;
    
    reg blink_state;
    parameter LED_BLINK_OFF = 0;
    parameter LED_BLINK_ON = 1;
    
    always @(posedge clock_25mhz)
        begin
            case (status_indicator)
                LIGHT_OFF:
                    begin
                        LED_0_input <= 0;
                    end
                BLINKING:   
                    begin
                        case (blink_state)
                            LED_BLINK_OFF:
                                begin
                                    LED_0_input <= 0;
                                    if (one_hz_enable)
                                        begin
                                            blink_state <= LED_BLINK_ON;
                                        end
                                end
                            LED_BLINK_ON:
                                begin
                                    LED_0_input <= 1;
                                    if (one_hz_enable)
                                        begin
                                            blink_state <= LED_BLINK_OFF;
                                        end
                                end 
                        endcase
                    end
                CONSTANTLY_ON:
                    begin
                        LED_0_input <= 1;
                    end
            endcase 
        end
                        
endmodule                 

module siren_generator(input alarm,
                    input clock_25mhz,
                    output reg sound);

    reg state;
    parameter LOW_TO_HIGH = 0;
    parameter HIGH_TO_LOW = 1;
    
    reg [15:0] counter;
    reg [15:0] count_till;
    reg [15:0] count_till_half;
    parameter COUNT_TILL_440 = 56818; //number of clock ticks for a 440 hz frequency
    parameter COUNT_TILL_440_HALF = 28409; //half of the above amount
    parameter COUNT_TILL_880 = 28409; //number of clock ticks for a 880 hz frequency
    parameter COUNT_TILL_880_HALF = 14204; //half of the above amount
    reg [1:0] seconds_count;
    always @(posedge clock_25mhz)
        begin
            
            if (alarm)
                begin
                    case (state)
                        LOW_TO_HIGH: //go from 440 hz to 880 hz
                            begin
                                if (counter == count_till)
                                    begin
                                        counter <= 0;
                                        count_till <= count_till-20; //decrement by 20 -- arbitrary value to make it go to 880 fast enough
                                        count_till_half <= count_till_half-10;

                                    end
                                else
                                    begin
                                        counter <= counter + 1;
                                    end
                                if (counter < count_till_half) //first half of period, sound is on
                                    begin
                                        sound <= 1;
                                    end
                                else  //second half of period, sound is off
                                    begin
                                        sound <= 0;
                                    end
                                if (count_till <= 20)
                                    begin
                                        state <= HIGH_TO_LOW;
                                        counter <= 0;
                                        count_till <= COUNT_TILL_880;
                                        count_till_half <= COUNT_TILL_880_HALF;
                                    end
                            end
                        HIGH_TO_LOW: //go from 880 hz to 440 hz
                            begin
                                if (counter == count_till)
                                    begin
                                        counter <= 0;
                                        count_till <= count_till+20;
                                        count_till_half <= count_till_half+10;
                                    end
                                else
                                    begin
                                        counter <= counter + 1;
                                    end
                                if (counter < count_till_half)
                                    begin
                                        sound <= 1;
                                    end
                                else
                                    begin
                                        sound <= 0;
                                    end
                                if (count_till >= COUNT_TILL_440)
                                    begin
                                        state <= LOW_TO_HIGH;
                                        counter <= 0;
                                        count_till <= COUNT_TILL_440;
                                        count_till_half <= COUNT_TILL_440_HALF;
                                    end
                            end
                        default:
                            begin
                                state <= LOW_TO_HIGH;
                                counter <= 0;
                                count_till <= COUNT_TILL_440;
                                count_till_half <= COUNT_TILL_440_HALF;
                            end
                    endcase
                    
                end
            else
                begin
                    sound <= 0;
                end
        end         
                          
endmodule