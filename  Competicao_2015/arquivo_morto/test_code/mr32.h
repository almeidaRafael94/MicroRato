// ****************************************************************************
// MR32.H
//
// Current version: 
//  - 0.1 - 21/10/2011
//  - 0.2 - 22/04/2012
//  - 0.3 - 02/05/2012
//  - 0.4 - 15/07/2012
//  - 1.0 - 11/10/2012
//  - 1.1 - 10/11/2012
//  - 1.2 - 18/11/2012
//  - 1.3 - 21/11/2012
//
// J.L.Azevedo, DETI-UA
// ****************************************************************************

#ifndef __MR32_H
#define __MR32_H

#include <detpic32.h>

typedef unsigned int BOOL;

#define TRUE	1
#define FALSE	0

#define forever while(1)
#define and		&&
#define not		!
#define or		||

#define OBST_SENSOR_RIGHT	0
#define OBST_SENSOR_FRONT	1
#define OBST_SENSOR_LEFT	2
#define AN6					3
#define AN7					4
#define BATTERY				5
#define LINE_SENSOR			6

#define LINE_SENSOR_RIGHT2	0
#define LINE_SENSOR_RIGHT1	1
#define LINE_SENSOR_CENTER	2
#define LINE_SENSOR_LEFT1	3
#define LINE_SENSOR_LEFT2	4
#define LINE_SENSOR_ALL		5

#define end() return 0
#define waitStep40ms() while(!tick40ms); tick40ms = 0


#define startButton() (!PORTBbits.RB3)
#define stopButton() (!PORTBbits.RB4)

#define enableObstSens() LATBbits.LATB10=1
#define disableObstSens() LATBbits.LATB10=0

#define enableGroundSens() LATEbits.LATE5=1
#define disableGroundSens() LATEbits.LATE5=0

#define enableLineSens() LATEbits.LATE5=1
#define disableLineSens() LATEbits.LATE5=0

#define readBeaconSens() (PORTBbits.RB9)


// ****************************************************************************
// Global variables
typedef union
{
	struct 
	{
		int obst_sens_right;
		int obst_sens_front;
		int obst_sens_left;
		int battery;
		int an6;
		int an7;
		int line_sensor;
	};
	int array[7];
} MR32_SENS;

extern MR32_SENS sensors;

extern int SERVO_WIDTH_MIN;
extern int SERVO_WIDTH_MAX;

volatile BOOL tick10ms;
volatile BOOL tick20ms;
volatile BOOL tick40ms;
volatile BOOL tick80ms;
volatile BOOL tick160ms;

// ****************************************************************************
// Function prototypes
void initPIC32(void);
void setVel2(int leftSpeed, int rightSpeed);
void readSensors(void);
void setServoPos(int pos);
void setLed(int ledNr);
void resetLed(int ledNr);
void delay(unsigned int tenth_ms);
void wait(unsigned int tenth_seconds);

int obstacleSensor(unsigned int sensorId);
int lineSensor(unsigned int sensorId);
int sensor(unsigned int sensorId);
unsigned int battery(void);
void stopMotors(void);

void readEncoders(int *enc_m1, int *enc_m2);

#endif


