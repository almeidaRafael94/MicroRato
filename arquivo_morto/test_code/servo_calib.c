// ****************************************************************************
// SERVO_CALIB.C
//
// Current version: 0.1 - 11/10/2012
//
// J.L.Azevedo, DETI-UA
// ****************************************************************************
//
#include "mr32.h"


int menu(void);

int main(void)
{
	int ch, servoPos;

	initPIC32();

	printStr("\nMR32 - Servo Calibration\n");
	printStr("\nEnter: L(eft)/C(enter)/R(ight), +/-)\n");

	printStr("\nMIN: ");
	printInt10(SERVO_WIDTH_MIN);
	printStr("\nMAX: ");
	printInt10(SERVO_WIDTH_MAX);
	servoPos = 0;
	setServoPos(servoPos);
	while((ch=inkey()) != 0x1B)
	{
		if(ch == 'r' || ch == 'R')
			setServoPos((servoPos=15));
		if(ch == 'l' || ch == 'L')
			setServoPos((servoPos=-15));
		if(ch == 'c' || ch == 'C')
			setServoPos((servoPos=0));
		if(ch == '+')
		{
			if(servoPos == -15)
				SERVO_WIDTH_MAX += 5;
			else
				SERVO_WIDTH_MIN -= 5;
			setServoPos(servoPos);
			printStr("\nMIN: ");
			printInt10(SERVO_WIDTH_MIN);
			printStr("\nMAX: ");
			printInt10(SERVO_WIDTH_MAX);
		}
		if(ch == '-')
		{
			if(servoPos == -15)
				SERVO_WIDTH_MAX -= 5;
			else
				SERVO_WIDTH_MIN += 5;
			setServoPos(servoPos);
			printStr("\nMIN: ");
			printInt10(SERVO_WIDTH_MIN);
			printStr("\nMAX: ");
			printInt10(SERVO_WIDTH_MAX);
		}
	}
	return 0;
}


