/PROG  CS5553_LB_LAB2                           # Header for program name
/ATTR                                           # Header for program attributes
OWNER		= ASCBIN;                              # Program owner, default
COMMENT		= "";                               # I had no comments on my program
PROG_SIZE	= 1684;                             # Program size (in bytes... pg 166 in manual) is just size of program file, not memory allocated... see MEMORY_SIZE comment
CREATE		= DATE 25-09-02  TIME 15:27:24;     # Creation date and time of the program 
MODIFIED	= DATE 25-09-02  TIME 16:37:20;        # Most recent date and time the program was displayed in the editor view
FILE_NAME	= ;                                 # Default blank
VERSION		= 0;                                # Version number (I only made one, so V0)
LINE_COUNT	= 37;                               # Number of lines of code 
MEMORY_SIZE	= 2176;                             # Size in memory (more than PROG_SIZE bc this is the actual memory allocated to run, not just the size of the code file)
PROTECT		= READ_WRITE;                       # Protections on this program, write protection was set to OFF so write is available
TCD:  STACK_SIZE	= 0,                          # Not specified by me, but default allocates 500 bytes at runtime (pg 170)
      TASK_PRIORITY	= 50,                      # The next few lines are default values for task coltrol data (TCD)... Fanuc proprietary stuff
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;                     # Group mask (identifies axes the program will control; pg 168) default is 1 and * indiciate unused for the rest
CONTROL_CODE	= 00000000 00000000;             # Internal control flags, default 0's
LOCAL_REGISTERS	= 0,0,0;                      # No local registers allocated for this program
/APPL                                           # Application-specific headers would go here (e.g. welding), but I have none so blank
/APPL

AUTO_SINGULARITY_HEADER;
  ENABLE_SINGULARITY_AVOIDANCE   : FALSE;       # Singularity avoidance is turned off so program will run exactly as told (not going to adjust wrist orientation)
/MN                                             # Main program starts
   1:L P[2] 250mm/sec FINE    ;                 # Linear move to local variable position 2 (home) at 250mm/s speed and stops there.. all my move commands have the same format just different locations
   2:  CALL IPL_OR_RG_MOVE(100,40,1) ;          # Open gripper: 40N, width of 100mm
   3:L P[3] 250mm/sec FINE    ;                 # Move to location 3 (above dice)
   4:L P[1] 250mm/sec FINE    ;                 # Move to location 1 (over dice)
   5:  CALL IPL_OR_RG_MOVE(77.6,40,1) ;         # Close gripper: 40N, width of 77.6mm
   6:L P[1] 250mm/sec FINE    ;                 # Move to 1
   7:L P[2] 250mm/sec FINE    ;                 # Move to 2 (home)
   8:L P[4] 250mm/sec FINE    ;                 # Move to 4 (above conveyor)
   9:L P[5] 250mm/sec FINE    ;                 # Move to 5 (so dice is on conveyor)
  10:  CALL IPL_OR_RG_MOVE(100,40,1) ;          # Open gripper
  11:L P[5] 250mm/sec FINE    ;                 # Move to 5 (unnecessary movement bc it should still be there)
  12:L P[4] 250mm/sec FINE    ;                 # Move to 4
  13:  LBL[1] ;                                 # Label 1 for jump point in the if on line 15
  14:  DO[138:CONVEYOR 1 REVERSE]=ON ;          # Turn conveyor 1 reverse on
  15:  IF (DI[139:CONVEYOR 1 RIGHT PROX]=ON) THEN ;   # If the proximity sensor is broken then...
  16:  DO[138:CONVEYOR 1 REVERSE]=OFF ;                  # Turn the conveyor off (to stop the dice)
  17:  ELSE ;                                         # Else (proximity sensor not broken)
  18:  JMP LBL[1] ;                                      # Go to label 1 to make this loop til the dice reaches the proximity sensor
  19:  ENDIF ;                                  # End of the if statement started on line 15
  20:L P[6] 250mm/sec FINE    ;                 # Move to 6 (Above stopped dice)
  21:L P[7] 250mm/sec FINE    ;                 # Move to 7 (Over stopped dice)
  22:  CALL IPL_OR_RG_MOVE(77.6,40,1) ;         # Close gripper
  23:L P[6] 250mm/sec FINE    ;                 # Move to 6 (Pick up dice)
  24:L P[8] 250mm/sec FINE    ;                 # Move to 8 (Rotate dice on the x-axis in the air)
  25:L P[9] 250mm/sec FINE    ;                 # Move to 9 (Set dice back down on conveyor)
  26:  CALL IPL_OR_RG_MOVE(100,40,1) ;          # Open gripper
  27:L P[8] 250mm/sec FINE    ;                 # Move back to 8 (untwist arm from rotating)
  28:L P[10] 250mm/sec FINE    ;                # Move to 10 (above new dice spot)
  29:L P[11] 250mm/sec FINE    ;                # Move to 11 (over dice)
  30:  CALL IPL_OR_RG_MOVE(77.6,40,1) ;         # Close gripper
  31:L P[10] 250mm/sec FINE    ;                # Move to 10 (somewhere in the air to avoid colliding with conveyor)
  32:L P[2] 250mm/sec FINE    ;                 # Move to 2 (home)
  33:L P[3] 250mm/sec FINE    ;                 # Move to 3 (above dice original spot)
  34:L P[1] 250mm/sec FINE    ;                 # Move to 1 (dice original spot)
  35:  CALL IPL_OR_RG_MOVE(100,40,1) ;          # Open gripper
  36:L P[3] 250mm/sec FINE    ;                 # Move to 3 (above dice)
  37:L P[2] 250mm/sec FINE    ;                 # Move to 2 (home)
/POS                                            # Local position variables (P[x] values)
P[1]{                                                                               # Dice original location; following attributes are the same for all locations
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, -1',                                    # User and Tool frame are both 1 (world), CONFIG is wrist configuration (flip, up/down, turn number)
	X =   628.230896  mm,	Y =      .380070  mm,	Z =    50.960602  mm,               # Cartesian coordinates of end of tool (EOT)
	W =  -179.912903 deg,	P =     1.924459 deg,	R =   179.394608 deg                # Yaw, Pitch, and Roll of arm
};
P[2]{                                                                               # Home
   GP1:
	UF : 1, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=     0.000000 deg,	J5=   -90.000000 deg,	J6=  -180.000000 deg
};
P[3]{                                                                               # Above dice original location
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, -1',
	X =   621.617432  mm,	Y =    -1.351940  mm,	Z =    97.204506  mm,
	W =   179.479996 deg,	P =     4.369098 deg,	R =   179.332123 deg
};
P[4]{                                                                               # Above conveyor
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, -1',
	X =  -137.841873  mm,	Y =  -618.357849  mm,	Z =   533.174805  mm,
	W =   179.267593 deg,	P =     2.096530 deg,	R =    88.393578 deg
};
P[5]{                                                                               # Closer so dice is on conveyor
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -173.302933  mm,	Y =  -598.991699  mm,	Z =   242.622101  mm,
	W =   179.373367 deg,	P =    -1.630008 deg,	R =    88.767609 deg
};
P[6]{                                                                               # Above dice after conveyor moved it
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -185.099304  mm,	Y =  -721.265381  mm,	Z =   301.047516  mm,
	W =   178.038376 deg,	P =     3.736897 deg,	R =    87.256889 deg
};
P[7]{                                                                               # Over dice after conveyor moved it
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -182.774704  mm,	Y =  -742.760803  mm,	Z =   245.036652  mm,
	W =   178.004669 deg,	P =     -.594133 deg,	R =    87.526825 deg
};
P[8]{                                                                               # Rotated position in the air
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, -1',
	X =  -157.385559  mm,	Y =  -442.853668  mm,	Z =   173.597610  mm,
	W =   104.984909 deg,	P =    87.094055 deg,	R =    12.638175 deg
};
P[9]{                                                                               # Rotated position above the conveyor
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, -1',
	X =  -159.193588  mm,	Y =  -452.688782  mm,	Z =     8.776377  mm,
	W =   167.914551 deg,	P =    82.602554 deg,	R =    75.247078 deg
};
P[10]{                                                                               # Above dice after rotating
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -165.832581  mm,	Y =  -709.791443  mm,	Z =   360.189728  mm,
	W =   179.580521 deg,	P =      .488540 deg,	R =    89.497421 deg
};
P[11]{                                                                               # On dice after rotating
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -169.616257  mm,	Y =  -739.613586  mm,	Z =   243.246979  mm,
	W =   179.508575 deg,	P =    -4.310607 deg,	R =    89.217552 deg
};
/END                                                                                 # End of program 