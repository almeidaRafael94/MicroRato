#include "mr32.h"

void StopRobot (void);
void mapTerrain(int, int);

int estado = 0;

int xx = 0;
int yy = 0;
int tt = 0;

int main(void)
{
	initPIC32();
	closedLoopControl(true);
	StopRobot();

	printf("Battery: %d \n", batteryVoltage());

	while(1)
	{
		StopRobot();

		if (batteryVoltage() <= 94)
		{
			printf("Bateria fraca, mudar bateria!!");
			StopRobot();
			return 0;
		}
		else
		{
			if (startButton() == 1)
			{
				estado = 1;
			}
		}
	
		while(estado == 1) // Robot arranca
		{
			if (stopButton() == 1)
			{
				StopRobot();
				estado = 0;
				printf("STOP BUTTON");
			}
			else
			{				
				setVel2(100, 100);

				getRobotPos(&x, &y, &t);
				printf("x:%f  y:%f  TETA:%f\n", x, y, t); // print pos
			
			}
		}

	}
	
}

void StopRobot (void)
{
	setVel2(0, 0);
	tick40ms = 0;
	while(tick40ms == 0);
	tick40ms = 0;
	while(tick40ms == 0);
	estado = 0;
}
