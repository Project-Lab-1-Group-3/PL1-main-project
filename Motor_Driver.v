`timescale 1ns / 1ps

module Motor_Driver(
    input 	clk,          		//100 MHz clock
    input 	[1:0] sw,     		//SW0-SW1
    input 	[1:0] JC,     		//JC Pmod port for current limiter input
    input 	[7:0] JB,     		//phototransistor and IPS inputs
    output 	[7:0] JA,    		//JA Pmod port for output to H-bridge
    output 	[3:0] an,    		//SSD anode
    output 	[6:0] seg, 		//SSD segments
    output 	[7:0] led,  		//LEDs for debugging
    output 	dp           		//decimal point is always off
);
	reg [7:0] 	JA_temp; 		//Used for output to the motors
	reg [6:0] 	seg_temp;		//Used for seven segment display output
	reg [1:0] 	JC_old = 2'b11;		//Remembers the previous JC input value
	reg [20:0] 	counter60 = 0;		//60 Hz counter for the motors
	reg [10:0] 	pulse_counter0, pulse_counter1, pulse_counter2, pulse_counter3;	//Count the number of pulses per 100 ms
	reg [23:0] 	pulse_timer = 0;	//10 Hz counter for the phototransistors
	reg [3:0] 	freq, old_pulse, last_direction;		
	reg 		stop = 0; 			//for current limiting
	parameter	F = 4'b0011, 			//Arbitrary numbers assigned to F and C parameters
				C = 4'b1100,
				FORWARD = 4'b1010, 	//parameters correspond to necesary JA outputs to move the motors
				BACK    = 4'b0101,
				LEFT    = 4'b1000,
				RIGHT   = 4'b0010,
				P_LEFT  = 4'b1001,
				P_RIGHT = 4'b0110,
				SPEED   = 1000000;	//PWM duty cycle parameter must be 0 - 1,666,666

	always @ (posedge clk) begin	//Every clock cycle

		if (counter60 < 1666666) counter60 = counter60 + 1;
		else counter60 = 0; //reset counter every 1,666,666 clock cycles
		
		//10 Hz counter to check frequency of phototransistor input
		if (pulse_timer < 10000000) begin              		//executes every clock cycle
			pulse_timer = pulse_timer + 1;
			if (old_pulse[0] != JB[0]) 			//if phototramsistor input has changed
			     pulse_counter0 = pulse_counter0 + 1;  	//increment appropriate pulse counter
			if (old_pulse[1] != JB[1]) 		
			     pulse_counter1 = pulse_counter1 + 1;
			if (old_pulse[2] != JB[2]) 
			     pulse_counter2 = pulse_counter2 + 1;
			if (old_pulse[3] != JB[3]) 
			     pulse_counter3 = pulse_counter3 + 1;
		end else begin  	//executes 10 times every second
			stop = 0;    	//every 100 ms reset this value to 0
			JC_old = JC; 	//save the value of JC for 100 ms
			//Cheching the values of the pulse counters and deciding if an LED has been detected
			//Frequency = 10 * counter / 2
			if ( 		pulse_counter0 < 12 && 
						pulse_counter1 < 12 && 
						pulse_counter2 < 12 && 
						pulse_counter3 < 12) freq = 0;	//All counters are under 60 Hz = no LED detedcted
			else if (  (pulse_counter0 > 12 && pulse_counter0 < 190) ||
					   (pulse_counter1 > 12 && pulse_counter1 < 190) ||
					   (pulse_counter2 > 12 && pulse_counter2 < 190) ||
					   (pulse_counter3 > 12 && pulse_counter3 < 190)) freq = F;	//If any pick up 60-950 Hz, it is friendly
			else if ( 	pulse_counter0 > 190 || 
						pulse_counter1 > 190 || 
						pulse_counter2 > 190 || 
						pulse_counter3 > 190) freq = C;	//If no friendlies are detected and >950 Hz is detected, it is criminal
						
			pulse_timer = 0; 	//resetting the 100 ms timer
			{pulse_counter0,pulse_counter1,pulse_counter2,pulse_counter3} = 0; //resetting pulse counters
		end
		
		old_pulse = JB[3:0];	//saving current phototransistor values to compare next clock cycle
		
		// Case statement to turn on the correct segments based on the contents of freq
		case (freq)
			F:       	seg_temp = 7'b0001110;		//Friendly
			C:       	seg_temp = 7'b1000110;		//Criminal
			default:	seg_temp = 7'b1111111;		//Off
		endcase

		//Line following logic
		if (counter60 < SPEED && stop == 0) begin	//SPEED parameter sets PWM duty cycle
		  if (JB[7:4] == 0) begin             		//all sensors active
		      JA_temp = last_direction;       		//keep turning the same direction
		  end else if (JB[7] == 0) begin      		//left sensor active
		      if (JB[6] == 0) begin           		//2 left most sensors active
		          JA_temp = LEFT;             		//gradual left turn
		          last_direction = LEFT;      		//remember direction
		      end else begin                  		//left sensor is active
		          JA_temp = P_LEFT;           		//pivot left
		          last_direction = LEFT;
		      end
		  end else if (JB[4] == 0) begin      		//right sensor is active
		      if (JB[5] == 0) begin           		//2 right most sensors are active
		          JA_temp = RIGHT;            		//gradual right turn
		          last_direction = RIGHT;
		      end else begin
		          JA_temp = P_RIGHT;          		//pivot right
		          last_direction = RIGHT;
		      end
		  end else if (JB[7:4] == 4'b1111) begin 	//no sensors are active (path is lost)
		      case (last_direction) 			//Continue turning in same direction as previous turn 
		          RIGHT: JA_temp = P_RIGHT; 		//switch to a pivot instead of a gradual turn
		          LEFT: JA_temp = P_LEFT;
        	   endcase
        	end else JA_temp = FORWARD;  			//go forward if no cases are true
		end else JA_temp = 0; 				//motors are off when stop = 1 or counter60 > SPEED
		
		//if statement stops the motors if current limiter is active for 100 ms
		if (JC != 2'b11 && JC_old != 3'b11) stop = 1; 
	end
	
    	//Assigning outputs
    	assign led[3:0] = JB[7:4];	//LEDs are used for debugging
    	assign led[5:4] = JC[1:0];
    	assign led[6] = stop;
	assign an = 4'b1110; 		//only the first digit of the seven segment display is used for now
	assign dp = 1;			//decimal point is always off
	assign JA = JA_temp;		//temp registers are used in the procedural code and assigned to outputs
	assign seg = seg_temp;
endmodule
