#include "mr32.h"

void StopRobot (void);
void mapTerrain(int, int);

int estado = 0;
int position [100][100]; // Tentar meter dinamico
int index = 0;

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
			printf("Mudar Bateria CARALHO!!");
			StopRobot();
		}

		if (startButton() == 1)
		{
			estado = 1;
		}

		while(estado == 1)
		{
			if (stopButton() == 1)
			{
				estado = 0;
				StopRobot();
				printf("STOP BUTTON");
			}
			else
			{	
				double x, y, t;

				setVel2(100, 100);
				wait(10);


				getRobotPos(&x, &y, &t);
				printf("x:%f  y:%f  TETA:%f\n", x, y, t);
				mapTerrain(x, y, index++);
			}
		}

	}
	
}

void StopRobot (void)
{
	setVel2(0, 0);
	//tick40ms = 0;
	//while(tick40ms == 0);
	//tick40ms = 0;
	//while(tick40ms == 0);
	estado = 0;
}
void mapTerrain(int x, int y, int index)
{
	position[index][index] = 

}
