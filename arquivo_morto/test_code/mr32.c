// ****************************************************************************
// MR32.C
//
// Current version: 
//  - 0.1 - 21/10/2011
//  - 0.2 - 11/04/2012
//  - 0.3 - 02/05/2012
//  - 0.4 - 15/07/2012
//  - 1.0 - 11/10/2012
//  - 1.1 - 10/11/2012
//  - 1.2 - 18/11/2012
//  - 1.3 - 21/11/2012
//  - 1.4 - 24/10/2013
//          added code to control robot velocity by using encoders info
//  - 1.5 - 25/10/2013
//          support for C++ compilation
//
// J.L.Azevedo, DETI-UA
// ****************************************************************************

#include "mr32.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IN  1
#define OUT 0

#define PI	3.141592654

#define ROBOT	11

void pid(int sp_m1, int enc_m1, int sp_m2, int enc_m2);
void readEncoders(int *enc_m1, int *enc_m2);
void actuateMotors(int velL, int velR);

// ****************************************************************************
// Servo calibarion values (these values must be adjusted for each servo)
#define SERVO_MIN_PWM_IS_RIGHT

#if(ROBOT == 10)
int SERVO_WIDTH_MIN = 595;	// 0.595 ms 
int SERVO_WIDTH_MAX = 2425; // 2.425 ms
#elif(ROBOT == 11)
int SERVO_WIDTH_MIN = 595;	// 0.595 ms 
int SERVO_WIDTH_MAX = 2515; // 2.515 ms
#elif(ROBOT == 9)
int SERVO_WIDTH_MIN = 535;	// 0.535 ms 
int SERVO_WIDTH_MAX = 2365; // 2.365 ms
#else
int SERVO_WIDTH_MIN = 700;	// 0.7 ms 
int SERVO_WIDTH_MAX = 2200; // 2.2 ms
#endif


// ****************************************************************************

#define M1_IN1 LATBbits.LATB5
#define M1_IN2 LATCbits.LATC13
#define M2_IN1 LATBbits.LATB13
#define M2_IN2 LATFbits.LATF3
#define STDBY  LATCbits.LATC14

#define LED1   LATEbits.LATE0
#define LED2   LATEbits.LATE1
#define LED3   LATEbits.LATE2
#define LED4   LATEbits.LATE3
#define LED5   LATBbits.LATB15

#define M1_FORWARD M1_IN1=1; M1_IN2=0
#define M1_REVERSE M1_IN1=0; M1_IN2=1

#define M2_FORWARD M2_IN1=0; M2_IN2=1
#define M2_REVERSE M2_IN1=1; M2_IN2=0

// ****************************************************************************
// Servo constants	(do not change any of these values)
//
#define SERVO_LEVELS 	(POS_RIGHT - POS_LEFT)
#define T2_FREQ 	625		// fin_t2 = 625 kHz
#define SERVO_K 	(((SERVO_WIDTH_MAX - SERVO_WIDTH_MIN) * T2_FREQ) / 1000) / SERVO_LEVELS
#define POS_LEFT	-15
#define POS_RIGHT	15

MR32_SENS sensors;
volatile BOOL tick10ms;
volatile BOOL tick20ms;
volatile BOOL tick40ms;
volatile BOOL tick80ms;
volatile BOOL tick160ms;


#ifdef USING_ENCODERS
	static int spRight=0, spLeft=0;
	static int counter_m1 = 0;
	static int counter_m2 = 0;
#else
	static int velRight=0, velLeft=0;
#endif

int avfilter_mleft(int vel);
int avfilter_mright(int vel);
unsigned int getLineSensors(void);

// ****************************************************************************
// initPIC32()
//
void initPIC32(void)
{
	int i;
// Disable JTAG
	DDPCON = 3;

// Config Timer2, Timer3, OC1, OC2 and OC5
	T2CONbits.TCKPS = 5;	// 1:16 prescaler (i.e. fin = 625 KHz)
	PR2 = 6249;				// Fout = 20M / (32 * (6249 + 1)) = 100 Hz
	TMR2 = 0;				// Reset timer T2 count register
	T2CONbits.TON = 1;		// Enable timer T2 (must be the last command of 
							// the timer configuration sequence)
							//
	T3CONbits.TCKPS = 4;	// 1:32 prescaler (i.e. fin = 1.25 MHz)
	PR3 = 63;				// Fout = 20M / (16 * (63 + 1)) = 20000 Hz
	TMR3 = 0;				// Reset timer T2 count register
	T3CONbits.TON = 1;		// Enable timer T2 (must be the last command of 
							// the timer configuration sequence)
// Motor1 PWM

	OC1CONbits.OCM = 6; 	// PWM mode on OCx; fault pin disabled
	OC1CONbits.OCTSEL =1;	// Use timer T3 as the time base for PWM generation
	OC1RS = 0;				// 
	OC1CONbits.ON = 1;		// Enable OC1 module

// Motor2 PWM
	OC2CONbits.OCM = 6; 	// PWM mode on OCx; fault pin disabled
	OC2CONbits.OCTSEL =1;	// Use timer T3 as the time base for PWM generation
	OC2RS = 0;				// 
	OC2CONbits.ON = 1;		// Enable OC2 module

// Servo PWM
	OC5CONbits.OCM = 6; 	// PWM mode on OCx; fault pin disabled
	OC5CONbits.OCTSEL =0;	// Use timer T2 as the time base for PWM generation
	OC5RS = 0;				// 
	OC5CONbits.ON = 1;		// Enable OC5 module

	IFS0bits.T2IF = 0;
	IPC2bits.T2IP = 1;
	IEC0bits.T2IE = 1;		// Enable Timer 2 interrupts
	IEC0bits.T2IE = 1;		
	setServoPos(0);
	EnableInterrupts();

// ****************************************************************************
// IO Config
//
//  1-Bridge control
	STDBY = 1;				// Half-Bridge ON

	M1_IN1 = M1_IN2 = 0;	// STOP
	M2_IN1 = M2_IN2 = 0;	// STOP

	TRISCbits.TRISC14 = OUT;	// STDBY
	TRISBbits.TRISB5 = OUT;  	// M1_IN1
	TRISCbits.TRISC13 = OUT; 	// M1_IN2

	TRISBbits.TRISB13 = OUT;  	// M2_IN1
	TRISFbits.TRISF3 = OUT;  	// M2_IN2

//  2-Leds
	LATECLR = 0x000F;		// Leds 4-1 OFF
	LATBCLR = 0x8000;		// Led 5 OFF
	TRISECLR = 0x000F;		// RE3-0 as output
	TRISBCLR = 0x8000;		// RB15 as output

//  3-Sensors
	LATBbits.LATB10 = 0;		// Disable Obstacle sensors output
	TRISBbits.TRISB10 = OUT;	// EN_OBST_SENS as output
	TRISBbits.TRISB9 = IN;		// IV BEACON as input

    LATECLR = 0x0020;			// Disable line sensor
	TRISEbits.TRISE5 = OUT;		// EN_GND_SENS as output

	LATD = LATD | 0x00EC;		// Line sensor: output latches must be set
	TRISD = TRISD & ~(0x00EC);	// Line sensor: 5 bits as output

//  4- start/stop buttons 
	CNPUE = CNPUE | 0x60;		// Activate weak pull-ups in input ports RB3 and RB4

// ADC Config
	AD1PCFGbits.PCFG0 = 0;		// RB0 configured as analog input (AN0)
	AD1PCFGbits.PCFG1 = 0;		// RB1 configured as analog input (AN1)
	AD1PCFGbits.PCFG2 = 0;		// RB2 configured as analog input (AN2)
	AD1PCFGbits.PCFG6 = 0;		// RB6 configured as analog input (AN6)
	AD1PCFGbits.PCFG7 = 0;		// RB7 configured as analog input (AN7)
	AD1PCFGbits.PCFG11 = 0;		// RB11 configured as analog input (AN11)

	AD1CON1bits.SSRC = 7;		// Conversion trigger: internal counter ends
								// sampling and starts conversion
	AD1CON1bits.CLRASAM = 1;	// Stop conversions when the 1st A/D converter
								// interrupt is generated. At the same time,
								// hardware clears the ASAM bit
	AD1CON3bits.SAMC = 16;		// Sample time is 16 TAD (TAD = 100 ns)
	AD1CON2bits.SMPI = 2 - 1;	// Interrupt is generated after 2 samples
	AD1CON1bits.ON = 1;			// Enable A/D converter

#ifdef USING_ENCODERS
// Encoders
	INTCONbits.INT1EP = 1;		// interrupt generated on rising edge
	INTCONbits.INT4EP = 1;		// interrupt generated on rising edge

	IPC1bits.INT1IP = 4;
	IPC4bits.INT4IP = 4;
	
	IFS0bits.INT1IF = 0;
	IFS0bits.INT4IF = 0;

	IEC0bits.INT1IE = 1;		// Enable INT1 interrupts
	IEC0bits.INT4IE = 1;		// Enable INT4 interrupts
	
	counter_m1 = counter_m2 = 0;
#endif

	// Fill in the battery array
	readSensors();
	for(i=0; i < 32; i++)
		battery();
}

// ****************************************************************************
// readSensors()
//
void readSensors(void)
{
	static int channels[]={0, 1, 2, 6, 7, 11};	// Do not change order...
	int i;

	for(i=0; i < 6; i++)
	{	
		AD1CHSbits.CH0SA = channels[i];	// analog channel
		AD1CON1bits.ASAM = 1;			// Start conversion
		while( IFS1bits.AD1IF == 0 );	// Wait until AD1IF = 1
		sensors.array[i] = (ADC1BUF0 + ADC1BUF1) / 2;
		IFS1bits.AD1IF = 0;
	}
	sensors.array[LINE_SENSOR] = getLineSensors();
}

// ****************************************************************************
// getLineSensors()
//
unsigned int getLineSensors(void)
{
	unsigned int sensValue;

// The reading of the line sensor can be done here. However, if long loops are being used
//  the integration process allows the capacitors to charge completely; in that case the
//  read value will be always 0.

//	sensValue = PORTD >> 2;
//	sensValue = (sensValue & 0x0003) | ((sensValue & 0x38) >> 1);

// discharge capacitors
    LATECLR = 0x0020;			// Disable line sensor
	LATD = LATD | 0x00EC;		// All 5 outputs set (just in case, set in initPIC32() )
	TRISD = TRISD & ~(0x00EC);	// 5 bits as output

	delay(1);					// Wait, discharging capacitors
// charge capacitors
	TRISD = TRISD | 0x00EC;		// 5 bits as input
    LATESET = 0x0020;			// Enable line sensor

	delay(50);					// wait 5 ms (6 ms)
								// this time is critical... capacitors take time to charge
								// too little time: output capacitors don't charge enough
	sensValue = PORTD >> 2;
	sensValue = (sensValue & 0x0003) | ((sensValue & 0x38) >> 1);
    LATECLR = 0x0020;			// Disable line sensor

	return sensValue;
}

// ****************************************************************************
// battery() - Read battery voltage (average of the last 32 readings)
//           - returned value is multiplied by 10 (max. value is 101, i.e. 10,1 V)
unsigned int battery(void)
{
	static unsigned char array[32];
	static int i = 0, sum = 0;
	int value;

	value = sensors.array[BATTERY];

	value = (value * 330 + 511) / 1023;
	value = (value * (3300 + 6800) + 16500) / 33000;

	sum = sum - array[i] + value;
	array[i] = (unsigned char)value;
	i = (i + 1) & 0x1F;

	return sum >> 5;
}




// ****************************************************************************
// avfilter_mleft()
//
int avfilter_mleft(int vel)
{
	static int i=0, sum=0;
	static int buf[4] = {0,0,0,0};

	sum = sum - buf[i] + vel;
	buf[i++] = vel;
	i &= 0x03;
	return sum / 4;
}

// ****************************************************************************
// avfilter_mright()
//
int avfilter_mright(int vel)
{
	static int i=0, sum=0;
	static int buf[4] = {0,0,0,0};

	sum = sum - buf[i] + vel;
	buf[i++] = vel;
	i &= 0x03;
	return sum / 4;
}

// ****************************************************************************
// setServoPos(int pos)
//
void setServoPos(int pos)
{
#ifdef SERVO_MIN_PWM_IS_LEFT
	pos = pos < POS_LEFT ? POS_LEFT : pos;
	pos = pos > POS_RIGHT ? POS_RIGHT : pos;
	pos += -POS_LEFT;	// PWM is minimum @ left position
#else
	pos = pos < POS_LEFT ? POS_LEFT : pos;
	pos = pos > POS_RIGHT ? POS_RIGHT : pos;
	pos = -pos;			// 
	pos += POS_RIGHT;	// PWM is minimum @ right position
#endif
	OC5RS = ((SERVO_WIDTH_MIN * T2_FREQ) / 1000  + pos * SERVO_K) + 1;
}

// ****************************************************************************
// setLed(int ledNr)
//
// ledNr[0..3]
void setLed(int ledNr)
{
	int val;

	ledNr = ledNr < 0 ? 0 : ledNr;
	ledNr = ledNr > 3 ? 3 : ledNr;

	val = 1;

	val = val << ledNr;
	LATE = LATE | val;
}

// ****************************************************************************
// resetLed(int ledNr)
//
void resetLed(int ledNr)
{	
	int val;

	ledNr = ledNr < 0 ? 0 : ledNr;
	ledNr = ledNr > 3 ? 3 : ledNr;

	val = 1;

	val = val << ledNr;
	LATE = LATE & ~val;
}

// ****************************************************************************
// Interrupt Service routine - Timer2
//
void _int_(_TIMER_2_VECTOR) isr_t2(void)
{
	static int cntT2Ticks = 0;
#ifdef USING_ENCODERS
	static int encLeft, encRight;
#endif

	cntT2Ticks++;
	tick10ms = 1;								// Set every 10 ms
	if((cntT2Ticks % 2) == 0) tick20ms = 1;		// Set every 20 ms
	if((cntT2Ticks % 4) == 0) tick40ms = 1;		// Set every 40 ms
	if((cntT2Ticks % 8) == 0) tick80ms = 1;		// Set every 80 ms
	if((cntT2Ticks % 16) == 0) tick160ms = 1;	// Set every 160 ms

#ifdef USING_ENCODERS
	readEncoders(&encLeft, &encRight);
	pid(spLeft, encLeft, spRight, encRight);	// spLeft, spRight are global vars
#else											// set by setSP2()
	actuateMotors(velLeft, velRight);			// velLeft, velRight are global vars
#endif											// set by SetVel2()

	IFS0bits.T2IF = 0;
}


#ifdef USING_ENCODERS
// ****************************************************************************
// Interrupt Service routine - External Interrupt 1 (encoder M1)
//
void _int_(_EXTERNAL_1_VECTOR) isr_enc_m1(void)
{
	if(PORTEbits.RE6 == 1)
		counter_m1++;
	else
		counter_m1--;

	IFS0bits.INT1IF = 0;

}

// ****************************************************************************
// Interrupt Service routine - External Interrupt 4 (encoder M2)
//
void _int_(_EXTERNAL_4_VECTOR) isr_enc_m2(void)
{
	if(PORTEbits.RE7 == 1)
		counter_m2++;
	else
		counter_m2--;

	IFS0bits.INT4IF = 0;
}
#endif


// ****************************************************************************
// delay() - input: value in 1/10 ms
//
void delay(unsigned int tenth_ms)
{
	tenth_ms = tenth_ms > 500000 ? 500000 : tenth_ms;

	resetCoreTimer();
	while(readCoreTimer() <= (2000 * tenth_ms));
}

// ****************************************************************************
// wait() - input: value in 1/10 s
//
void wait(unsigned int tenth_seconds)
{
	resetCoreTimer();
	while(readCoreTimer() <= (2000000 * tenth_seconds ));
}


#if 1
// ****************************************************************************
// stopMotors()
//
void stopMotors(void)
{
	setVel2(0, 0);
	tick40ms = 0;
	while(tick40ms == 0);
	tick40ms = 0;
	while(tick40ms == 0);
}
#endif

// ****************************************************************************
// obstacleSensor() - Read one specific obstacle sensor (for dummies, only ;)
//
int obstacleSensor(unsigned int sensorId)
{
	if(sensorId > 2) 
		sensorId = 2;
	return sensors.array[sensorId];
}


// ****************************************************************************
// sensor - Read any sensor
//
int sensor(unsigned int sensorId)
{
	if(sensorId > 6) 
		sensorId = 6;
	return sensors.array[sensorId];
}

// ****************************************************************************
// lineSensor() - Read one specific line sensor (for dummies, only ;)
//
int lineSensor(unsigned int sensorId)
{
	if(sensorId > 4) 
		return sensors.array[LINE_SENSOR];
	else
		return (sensors.array[LINE_SENSOR] >> sensorId) & 0x01;
}

unsigned int pow2(int exp)
{
	unsigned int res = 1;
	for(; exp > 0; exp--)
		res *= 2;
	return res;
}



double cosine(double x)
{
	double y, t=0.0, absx, frac, quad;

	static double p0 = 0.999999999781;
	static double p1 =-0.499999993585;
	static double p2 = 0.041666636258;
	static double p3 =-0.0013888361399;
	static double p4 = 0.00002476016134;
	static double p5 =-0.00000026051495;
	static double pi2=1.570796326794896; /* pi/2 */

	absx = x;
	if (x < 0)
		absx=-absx; /* absolute value of input */
	quad = (int) (absx/pi2); /* quadrant (0 to 3) */
	frac = (absx/pi2) - quad; /* fractional part of input */
	
	if(quad == 0) t = frac * pi2;
	if(quad == 1) t = (1-frac) * pi2;
	if(quad == 2) t = frac * pi2;
	if(quad == 3) t = (frac-1) * pi2;
	
	t = t * t;
	y = p0 + (p1*t) + (p2*t*t) + (p3*t*t*t) + (p4*t*t*t*t) + (p5*t*t*t*t*t);
	
	if(quad == 2 || quad == 1) 
		y=-y; /* correct sign */

	return(y);
}

double sine(double x)
{
	return cosine(x - PI / 2.0);
}


double normalizeAngle(double angle)
{
	while( angle > (2.0 * PI) )
		angle = angle / (2.0 * PI);

	angle = angle < -PI ? angle + 2.0 * PI : angle;
	angle = angle > PI  ? angle - 2.0 * PI : angle;
	return angle;
}

#ifdef USING_ENCODERS
// ****************************************************************************
// setSP2()
//
void setSP2(int spL, int spR)
{
	DisableInterrupts();
	spLeft = spL;
	spRight = spR;
	EnableInterrupts();
}

// ****************************************************************************
// readEncoders()
//
void readEncoders(int *enc_m1, int *enc_m2)
{
	DisableInterrupts();
	*enc_m1 = -counter_m1;
	*enc_m2 = counter_m2;
	counter_m1 = counter_m2 = 0;
	EnableInterrupts();
}

// ****************************************************************************
// PID controller. 
// 
// THIS FUNCTION SHOULD BE CALLED FROM A TIMER INTERRUPT SERVICE ROUTINE (T >= 10ms)
// DO NOT CALL THIS FUNCTION DIRECTLY FROM YOUR CODE (MAIN). DOING THAT MAY 
// PERMANENTLY DAMAGE THE MOTOR DRIVE.
//
void pid(int sp_m1, int enc_m1, int sp_m2, int enc_m2)
{
	int cmd_m1=0, cmd_m2=0;

	// Code for a PID, here
	// ...
	
	actuateMotors(cmd_m1, cmd_m2);	// Actuate directly on motors 
}									

#else

// ****************************************************************************
// setVel2()
//
void setVel2(int velL, int velR)
{
	DisableInterrupts();
	velLeft = velL;
	velRight = velR;
	EnableInterrupts();
}

#endif


// ****************************************************************************
// THIS FUNCTION SHOULD BE CALLED FROM A TIMER INTERRUPT SERVICE ROUTINE (T >= 10ms)
// DO NOT CALL THIS FUNCTION DIRECTLY FROM YOUR CODE (MAIN). DOING THAT MAY 
// PERMANENTLY DAMAGE THE MOTOR DRIVE.
//
void actuateMotors(int velL, int velR)
{
	velL = velL > 100 ? 100 : velL;
	velL = velL < -100 ? -100 : velL;

	velR = velR > 100 ? 100 : velR;
	velR = velR < -100 ? -100 : velR;
	if(velL < 0)
	{
		velL = -velL;
		M1_REVERSE;
	}
	else
	{
		M1_FORWARD;
	}

	if(velR < 0)
	{
		velR = -velR;
		M2_REVERSE;
	}
	else
	{
		M2_FORWARD;
	}
	OC1RS = ((PR3+1)*velL) / 100;
	OC2RS = ((PR3+1)*velR) / 100;
}	

void __gxx_personality_v0(void) { exit(1); }

#ifdef __cplusplus
}
#endif

