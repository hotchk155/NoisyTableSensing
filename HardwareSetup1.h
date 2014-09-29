// SETTINGS FOR PUBLIC 


// Digital inputs pins from the sensor trigger
// circuit for first side of the table
#define P_1A        14
#define P_1B        15
#define P_1C        16
#define P_1D        17

// Digital inputs pins from the sensor trigger
// circuit for second side of the table
#define P_2A        10
#define P_2B        8
#define P_2C        9
#define P_2D        11

// Bit masks used in the port registers for
// reading the sensors
#define BITMASK_1A  0x08
#define BITMASK_1B  0x04
#define BITMASK_1C  0x02
#define BITMASK_1D  0x01
#define BITMASK_1ALL (BITMASK_1A|BITMASK_1B|BITMASK_1C|BITMASK_1D)

#define BITMASK_2A  0x01
#define BITMASK_2B  0x02
#define BITMASK_2C  0x08
#define BITMASK_2D  0x04
#define BITMASK_2ALL (BITMASK_2A|BITMASK_2B|BITMASK_2C|BITMASK_2D)

// Timing: Timeout for a misread (clock ticks)
#define TIMER_TIMEOUT1 200 
#define TIMER_TIMEOUT2 250 

// Define the dimension of the table in sensor timing 
// counts
#define WIDTH1             17
#define HEIGHT1            13
#define SCALE1             10

#define WIDTH2             17
#define HEIGHT2            12
#define SCALE2             12

