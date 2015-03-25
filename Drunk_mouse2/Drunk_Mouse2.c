#include "mr32.h"

void StopRobot (void);

int estado;

int main(void)
{
	
	double x, y, t;

	initPIC32();
	closedLoopControl(true);
	StopRobot();

	setVel2(100, 100);
	wait(10);


	getRobotPos(&x, &y, &t);
	printf("x:%f  y:%f  TETA:%f\n", x, y, t);
	


}

void StopRobot ()
{
	setVel2(0, 0);
	tick40ms = 0;
	while(tick40ms == 0);
	tick40ms = 0;
	while(tick40ms == 0);
	estado = 0;
}