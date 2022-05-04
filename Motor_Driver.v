`timescale 1ns / 1ps

module Motor_Driver(
    input 	clk,          		//100 MHz clock
    input 	[15:0] sw,     		//SW0-SW1
    input 	[1:0] JC,     		//JC Pmod port for current limiter input
    input 	[7:0] JB,     		//phototransistor and IPS inputs
    output 	[7:0] JA,    		//JA Pmod port for output to H-bridge
    output 	[3:0] an,    		//SSD anode
    output 	[6:0] seg, 		  	//SSD segments
    output 	[7:0] led,  		//LEDs for debugging
    output 	dp           		//decimal point is always off
);
	reg [7:0] 	JA_temp; 			   	//Used for output to the motors
	reg [6:0] 	seg_temp;			   	//Used for seven segment display output
	reg [1:0] 	JC_old = 2'b11;			//Remembers the previous JC input value
	reg [20:0] 	counter60, counter50;  	//50 and 60 Hz counters for servos and motors
	reg [16:0]  counter1000;           	//1000 Hz counter for delaying
	reg         right_delay = 0, left_delay = 0; //used to delay the servos
	reg [17:0]  angle1 = 90000, angle2 = 210000; //servo angle duty cycle must be 90,000 - 210,000
	reg [10:0] 	pulse_counter0, pulse_counter1;	//Count the number of pulses per 100 ms
	reg [25:0] 	pulse_timer = 0;	  	//10 Hz counter for the phototransistors
	reg [3:0] 	an_temp, left_freq, right_freq, old_pulse, last_direction = 0;
	reg [3:0]   left_freq_old, right_freq_old;
	reg [1:0] 	ammo = 2'b11;
	reg [2:0]   shooting_delay = 0;
	reg [20:0]  SPEED = 0;
	reg 		stop = 0; 			   	//for current limiting
	parameter	F = 4'b0011, 		  	//Arbitrary numbers assigned to F and C parameters
				C = 4'b1100,
				FORWARD = 4'b1010, 		//parameters correspond to necesary JA outputs to move the motors
				BACK    = 4'b0101,
				LEFT    = 4'b1000,
				RIGHT   = 4'b0010,
				P_LEFT  = 4'b1001,
				P_RIGHT = 4'b0110;
				
	always @ (posedge clk) begin	//Every clock cycle

		if (counter60 < 1666666) counter60 = counter60 + 1;
		else counter60 = 0; //reset counter every 1,666,666 clock cycles
		
		if (counter50 < 2000000) counter50 = counter50 + 1;
		else counter50 = 0; //reset counter every 2,000,000 clock cycles
		
		//2 Hz counter to check frequency of phototransistor input
		if (pulse_timer < 50000000) begin              	//executes every clock cycle
			pulse_timer = pulse_timer + 1;
			if (old_pulse[0] != JB[0]) 					//if phototransistor input has changed
			     pulse_counter0 = pulse_counter0 + 1;  	//increment appropriate pulse counter
			if (old_pulse[1] != JB[1]) 		
			     pulse_counter1 = pulse_counter1 + 1;
		end else begin  	//executes 2 times every second
			stop = 0;    	//every 500 ms reset this value to 0
			JC_old = JC; 	//save the value of JC for 500 ms
			//Cheching the values of the pulse counters and deciding if an LED has been detected
			//Frequency = counter value
			if 			(pulse_counter0 < 100) 	left_freq = 0;	//left counter is less than 100 Hz
			else if 	(pulse_counter0 < 950)	left_freq = F;	//left counter is 100-950 Hz
			else if		(pulse_counter0 > 950)	left_freq = C;	//left counter is >950 Hz
			if 			(pulse_counter1 < 100) 	right_freq = 0;	//same thing for the right side
			else if 	(pulse_counter1 < 950)	right_freq = F;
			else if		(pulse_counter1 > 950)	right_freq = C;
			pulse_timer = 0; 	//resetting the 100 ms timer
			{pulse_counter0,pulse_counter1,right_delay,left_delay} = 0; //resetting pulse counters
						
			if (shooting_delay > 0) shooting_delay = shooting_delay - 1;	//used to prevent the capture system from shooting to frequently
		end
		
		old_pulse = JB[3:0];	//saving current phototransistor values to compare next clock cycle
		
		//Line following logic
		if (counter60 < SPEED && stop == 0 && sw[0] == 1) begin	//SPEED register sets PWM duty cycle
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
		      case (last_direction) 				//Continue turning in same direction as previous turn 
		          RIGHT: JA_temp = P_RIGHT; 		//switch to a pivot instead of a gradual turn
		          LEFT: JA_temp = P_LEFT;
        	   endcase
        	end else JA_temp = FORWARD;  			//go forward if no cases are true
		end else JA_temp = 0; 						//motors are off when stop = 1 or counter60 > SPEED
		
		//if statement stops the motors if current limiter is active for 500 ms
		if (JC != 2'b11 && JC_old != 3'b11) stop = 1; 
		
		//50 Hz PWM for the 2 servos
		if (counter50 < angle1 && right_delay == 0)	JA_temp[4] = 1; //angle1 controls right servo
		else JA_temp[4] = 0;
		if (counter50 < angle2 && left_delay == 0)	JA_temp[5] = 1; //angle2 controls left servo
		else JA_temp[5] = 0;
		
		// Case statement for the seven-segment display
		case (counter60[18:17])
			0: begin
				an_temp = 4'b1110;
				if (right_freq != 0) begin
					case (right_freq)
						F:       	seg_temp = 7'b0001110;		//Friendly
						C:       	seg_temp = 7'b1000110;		//Criminal
						default:	seg_temp = 7'b0111111;		//Off
					endcase
				end else if (left_freq != 0) begin
					case (left_freq)
						F:       	seg_temp = 7'b0001110;		//Friendly
						C:       	seg_temp = 7'b1000110;		//Criminal
						default:	seg_temp = 7'b0111111;		//Off
					endcase
				end else seg_temp = 7'b0111111;
			end
			1: begin
				an_temp = 4'b1101;
				case (ammo)
					0:			seg_temp = 7'b1000000;
					1:			seg_temp = 7'b1111001;
					2:			seg_temp = 7'b0100100;
					3:			seg_temp = 7'b0110000;
					default:	seg_temp = 7'b0111111;
				endcase
			end
			default:	seg_temp = 7'b1111111;
		endcase
	
	//shooting sequences
	if (right_freq != right_freq_old && right_freq == C && ammo != 0 && shooting_delay == 0 && sw[0] == 1) begin
	   angle1 = angle1 + 40000;    //increase right servo angle
	   angle2 = angle2 - 40000;    //decrease left servo angle
	   shooting_delay = 6;         //prevent the rover from shooting for 3 sec
	   right_delay = 1;            //delay the right servo for 500 ms
	   ammo = ammo - 1;            //adjust the ammo count
	end	
	if (left_freq != left_freq_old && left_freq == C && ammo != 0 && shooting_delay == 0 && sw[0] == 1) begin
	   angle1 = angle1 + 40000;    //increase right servo angle
	   angle2 = angle2 - 40000;    //decrease left servo angle
	   shooting_delay = 6;         //prevent the rover from shooting for 3 sec
	   left_delay = 1;             //delay the left servo for 500 ms
	   ammo = ammo - 1;            //adjust the ammo count
	end	
	
    right_freq_old = right_freq;   //remember the current frequency detected
    left_freq_old = left_freq;
	
	case (sw[3:1])				//switches 1-3 are used to select a speed setting
	   3'b001: SPEED = 1000000;	//60% speed
	   3'b010: SPEED = 1333333;	//80%
	   3'b100: SPEED = 1666666;	//100%
	endcase
	end
	
	//Assigning outputs
    assign led[3:0] = JB[7:4];	//LEDs are used for debugging
    assign led[5:4] = JC[1:0];
    assign led[6] = stop;
	assign an = an_temp;		//seven segment display annodes
	assign dp = 1;				//decimal point is always off
	assign JA = JA_temp;		//temp registers are used in the procedural code and assigned to outputs
	assign seg = seg_temp;
endmodule