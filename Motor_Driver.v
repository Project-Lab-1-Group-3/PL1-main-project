`timescale 1ns / 1ps

module Motor_Driver(
    input clk,          //100 MHz clock
    input [1:0] sw,     //SW0-SW1
    input [1:0] JC,     //JC Pmod port for current limiter input
    input [7:0] JB,     //phototransistor and IPS inputs
    output [7:0] JA,    //JA Pmod port for output to H-bridge
    output [3:0] an,    //SSD anode
    output [6:0] seg,   //SSD segments
    output [7:0] led,   //LEDs for debugging
    output dp           //decimal point is always off
);
	reg [7:0] JA_temp; 
	reg [6:0] seg_temp;
	reg [1:0] JC_old;
	reg [20:0] counter60 = 0;   	//for the motors
	reg [10:0] pulse_counter0, pulse_counter1, pulse_counter2, pulse_counter3;	//Counts number of pulses per 100 ms
	reg [23:0] pulse_timer = 0; 	//10 Hz
	reg [3:0] freq, old_pulse, last_direction;		
	reg stop = 0; //for current limiting
	parameter	F = 4'b0011, //Arbitrary numbers assigned to F and C
				C = 4'b1100,
				FORWARD = 4'b1010, //parameters correspond to necesary JA outputs to move the motors
				BACK    = 4'b0101,
				LEFT    = 4'b1000,
				RIGHT   = 4'b0010,
				P_LEFT  = 4'b1001,
				P_RIGHT = 4'b0110,
				SPEED   = 1666666; //100% PWM

	always @ (posedge clk) begin

		if (counter60 < 1666666) counter60 = counter60 + 1;
		else counter60 = 0;
		//10 Hz counter to check frequency of phototransistor input
		if (pulse_timer < 10000000) begin              //executes every clock cycle
			pulse_timer = pulse_timer + 1;
			if (old_pulse[0] != JB[0]) 
			     pulse_counter0 = pulse_counter0 + 1;  //incrementing pulse counters
			if (old_pulse[1] != JB[1]) 
			     pulse_counter1 = pulse_counter1 + 1;
			if (old_pulse[2] != JB[2]) 
			     pulse_counter2 = pulse_counter2 + 1;
			if (old_pulse[3] != JB[3]) 
			     pulse_counter3 = pulse_counter3 + 1;
		end else begin  //executes 10 times every second
			if ( 		pulse_counter0 < 12 && 
						pulse_counter1 < 12 && 
						pulse_counter2 < 12 && 
						pulse_counter3 < 12) freq = 0;	//0-60 Hz
			else if (  (pulse_counter0 > 12 && pulse_counter0 < 190) ||
					   (pulse_counter1 > 12 && pulse_counter1 < 190) ||
					   (pulse_counter2 > 12 && pulse_counter2 < 190) ||
					   (pulse_counter3 > 12 && pulse_counter3 < 190)) freq = F;	//60-950 Hz
			else if ( 	pulse_counter0 > 190 || 
						pulse_counter1 > 190 || 
						pulse_counter2 > 190 || 
						pulse_counter3 > 190) freq = C;	// >950 Hz
			pulse_timer = 0; //resetting the 100 ms timer
			{pulse_counter0,pulse_counter1,pulse_counter2,pulse_counter3} = 0; //resetting pulse counters
			stop = 0; //every 100ms reset this value to 0
			JC_old = JC; //save the value of JC for 100 ms
		end
		old_pulse = JB[3:0];	//saving current value to compare next clock cycle
		
		// Case statement to turn on the correct segments based on the contents of sseg
		case (freq)
			F:       	seg_temp = 7'b0001110;		//F
			C:       	seg_temp = 7'b1000110;		//C
			default:	seg_temp = 7'b1111111;		//off
		endcase

		//Line following logic
		if (counter60 < SPEED && stop == 0) begin	//SPEED parameter sets PWM duty cycle
		  if (JB[7:4] == 0) begin             //all sensors active
		      JA_temp = last_direction;       //keep turning the same direction
		  end else if (JB[7] == 0) begin      //left sensor active
		      if (JB[6] == 0) begin           //2 left most sensors active
		          JA_temp = LEFT;             //gradual left turn
		          last_direction = LEFT;      //remember direction
		      end else begin                  //left sensor is active
		          JA_temp = P_LEFT;           //pivot left
		          last_direction = LEFT;
		      end
		  end else if (JB[4] == 0) begin      //right sensor is active
		      if (JB[5] == 0) begin           //2 right most sensors are active
		          JA_temp = RIGHT;            //gradual right turn
		          last_direction = RIGHT;
		      end else begin
		          JA_temp = P_RIGHT;          //pivot right
		          last_direction = RIGHT;
		      end
		  end else if (JB[7:4] == 4'b1111) begin //no sensors are active (path is lost)
		      case (last_direction) //Continue turning in same direction as previous turn 
		          RIGHT: JA_temp = P_RIGHT; //switch to a pivot instead of a gradual turn
		          LEFT: JA_temp = P_LEFT;
        	   endcase
        	end else JA_temp = FORWARD;  //go forward if none are true
		end else JA_temp = 0; //sets the PWM duty cycle
		
		//if statement stops the motors if current limiter is active for 200 ms
		if (JC != 2'b11 && JC_old != 3'b11) stop = 1; 
	end
	
    assign led[3:0] = JB[7:4]; 	//first 3 LEDs show IPS states
    assign led[5:4] = JC[1:0];
    assign led[6] = stop;
	assign an = 4'b1110; 		//only the 1st digit of the 7 seg is used
	assign dp = 1;				//decimal point is always off
	assign JA = JA_temp;
	assign seg = seg_temp;
endmodule
