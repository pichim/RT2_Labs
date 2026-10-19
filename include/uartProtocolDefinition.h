#pragma once

/*
-------- DATA PROTOCOL----------------------------
   254	 1	255	201	1	  4	     0		...   xx	   10 13
    n1	n2	rec id1	id2	#Byte1 #Byte2 thedata checksum CRLF("\r\n") terminator
--------------------------------------------------
1-20 sensor values,
		id1		id2
		10			 	Counter values
				1		c1
				2		c2	(increments)
		11		actual current
				1		i1 / A
				2		i2 / A
--------------------------------------------------
// NOT USED: 21-40 cntrl values,		21		desired values			1 		Phi1 / rad				2 		Phi2 / rad				3 		x / mm				4 		y / mm
--------------------------------------------------
101-120 estimates/actual values,
		101		angles and calculated x,y
				1 		Phi1 / rad
				2 		Phi2 / rad
				3		x / mm
				4		y / mm
				4		z / mm
				6		y / mm
        111     1       phi1,phi2,x,y,sens(state)
--------------------------------------------------
121-140 send techn. values, like offsets
		id1		id2
		121
				1 		inc_offset phi1 / increments	int16_t
				2		inc_offset phi2	/ 		"			"
				3 		inc_additional_offset phi1 / increments	int16_t
				4		inc_additional_offset phi2	/ 		"			"
		125
				1		num_it of X2P trafo		"			"
		131
				1		debug var (1,2,3,4)
--------------------------------------------------
141-160 send operational. values, like states
		id1		id2
		141
				1 		active state
				2 		selected motor id
				3 		is_on_trajectory
--------------------------------------------------
2xx: set-value and commands (like disable controller...)
		id1		id2

		202:			set desired absolute values
				1		phi1		rad		float
				2		phi2		rad		float
				3		x			mm		float
				4		y			mm		float
		203				Increment values
				1		dphi1		rad		float
				2		dphi2		rad		float
				3		dx			mm		float
				4		dy			mm		float
		204:			set desired absolute value touples
			    1		phi1, phi2	rad		float
			    2		x, x	    mm		float
			    3		fHz, amp1, amp2     float
			    4		x, y, z	    mm		float
		209:
				1       state request (in state machine)
				2       select motor
        210:    Log data
                100     log status
                101     start log
                1       send log values
        211     Log type
                1   step
                2   sine
		220			Laser on/off
				1	0 = off, 1 = on
		221			Toggle blaster
				1	0 = off, 1 = toggle
		230			external control on/off
				1	0 = off, 1 = on
		231			set new trajectory
				1	set interpolation method (0 = line, 1 = curve)
				2	set (phi1,phi2,dhphi1,dphi2)   not implemented!
				3	set (x,y,dx,dy)
		232			set new Point2Point Values
				1	set (phi11,phi21,phi12,phi22, vel, cycle method [1=single,2=Sine,3=Zigzag])
		233			set new Cone Values
				1	set (phi1,phi2,radius,sigma0, vel, cycle method [1=single,2=inf])
		241
				1 Send text
		250
				1 GPA message
*/


#define UART_BUF_LEN      	256 // max 256
#define UART_HEAD_LEN      	7   //
#define UART_TAIL_LEN       3   // csm + "\r" + "\n"
#define UART_DATA_LEN      	(UART_BUF_LEN-UART_HEAD_LEN-UART_TAIL_LEN)
#define UART_ERR_LEN        2

#define UART_HEAD_BY0       254
#define UART_HEAD_BY1       1
#define UART_HEAD_BY2       255
#define PACK_SIZE           200

#define UART_TERM_BY0       '\r'
#define UART_TERM_BY1       '\n'
