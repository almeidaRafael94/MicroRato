#include "mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// VARIABLES
#define LIMIAR 400
#define Velocidade 70

#define TRUE 1
#define FALSE 0



volatile int sensor_dir,sensor_esq,sensor_frente; 
volatile int estado=0;     /* 0 = parado; 1 = Run_Beacon; 2 = Go_Home; 3 = End */
volatile int linha = 0;
volatile int farolsen = 0;
volatile int reset = 0;
int ciclos=0;				//tempo em funcionamento

/********************************************/
// FUNCTIONS

void Stop_robot(void);
void Ajusta_Dir(void);
void Ajusta_Esq(void);
void Vira_Dir (void);
void Rodar_Sobre_Si_Esq (void);
void Rodar_Sobre_Si_Dir (void);
void Rodar_Sobre_Si (void);
void andar_frente (void);
void Vira_esq (void);
void Chegada_Farol (void);
void Ver_Farol(void);
void Run_Beacon(void);
void Fim(void);  
void TimeOut(void);
void rotateRel_naive(double deltaAngle);
/********************************************/
int main (void)
{
	int countCiclos = 100;
	//iniciçao da pic
  	initPIC32 ();
  	closedLoopControl( true );
  	setVel2(0, 0);


	printStr(__FILE__); // para saber o nome do ficheiro que esta a correr no robot
	printf("\r    battery: %d ", batteryVoltage());
	if(batteryVoltage() <94){
		printf("_Bateria fraca, MUDAR Bateria\n");

	}
	printStr("\n");
	enableObstSens();
	while(1)
	{	
		while(!tick40ms);
		tick40ms=0;
		readAnalogSensors();

		//state buttons

		if(startButton() == 1) 				// Botao start(preto) primido
		{
			estado = 1;
			leds(0x0);						//leds off
			countCiclos = 100;
		}

		else if(stopButton() == 1) 			//Botao stop(vermelho) primido
		{
			estado = 0;
			disableObstSens();
		}

		if(estado == 1) 
		{
			TimeOut();						// timeOut => tb devia ir para uma inturrupcao
			Chegada_Farol();
			if(countCiclos++ >= 75)
			{

				Ver_Farol();
				countCiclos = 0;
			}
			Run_Beacon();
		}

		if(estado == 2)
		{
			leds(0x1);
			Stop_robot();
		}

		if(estado == 3)
			Fim();

		else if(stopButton() == 1 || estado == 0)		// deslica o funcionamento, nenhum led activo
		{
			Stop_robot();
		}
 	}
  return (0);
}
//############################################################################
/* conficoraçao dos motores*/

void Stop_robot()  //serve para quando se carrega no botao de desligar
{
	setVel2(0,0);		// desliga os metores
	estado = 0;
}
//################
void Ajusta_Esq ()
{
	setVel2 (Velocidade-20,Velocidade );
}
//################
void Ajusta_Dir ()
{
	setVel2 (Velocidade,Velocidade-20);
}
//################
void Vira_Dir ()
{
	setVel2 (Velocidade-30, Velocidade+10); // Velocidade-40, Velocidade
}
//################
void Vira_esq ()
{
	setVel2 (Velocidade+10, Velocidade-30); // Velocidade+10, Velocidade-30
}
//################
void andar_frente ()
{
	setVel2 (Velocidade, Velocidade); // ALTERAR ISTO
}
//################
void Rodar_Sobre_Si_Dir()
{
	setVel2 (Velocidade, -Velocidade);
}
void Rodar_Sobre_Si_Esq()
{
	setVel2 (-Velocidade, Velocidade);
}
void Rodar_Sobre_Si()
{
	setVel2 (Velocidade, -Velocidade);
}
int randomDecisionRotate()
{
	int value = rand()%10;
	if(value <= 5)
		return 0;
	else
		return 1;
}
void stop_Motors()
{
	setVel2(0,0);
}

//#############################################################################
void Run_Beacon ()
{
	disableObstSens();
	enableObstSens();
	if(obstacleSensor(OBST_SENSOR_FRONT) > LIMIAR)
	{	
		stop_Motors();
		if(randomDecisionRotate() == 0)
		{	
			Rodar_Sobre_Si_Dir();
			wait(2);
		}
		else 
		{
			Rodar_Sobre_Si_Esq();
			wait(2);
		}
	}
	if(obstacleSensor(OBST_SENSOR_RIGHT) > LIMIAR)
	{
		Vira_Dir();
		
	}
	if(obstacleSensor(OBST_SENSOR_LEFT) > LIMIAR)
	{
		Vira_esq();
		
	}
	if(obstacleSensor(OBST_SENSOR_FRONT) < LIMIAR && obstacleSensor(OBST_SENSOR_RIGHT) < LIMIAR && obstacleSensor(OBST_SENSOR_LEFT)< LIMIAR)
	{	
		andar_frente();
	}
	if(obstacleSensor(OBST_SENSOR_FRONT) > LIMIAR && obstacleSensor(OBST_SENSOR_RIGHT) > LIMIAR && obstacleSensor(OBST_SENSOR_LEFT)> LIMIAR)
	{	
		setVel2(0,0);
		Rodar_Sobre_Si();
		wait(1);
	}
}
//###################################################################################
void Chegada_Farol ()
{	
	int detectLine = 0;
	if( readLineSensors(0) > 5)
	{		
		while(readLineSensors(0) > 10 && detectLine <= 5)
		{	
		
			detectLine ++;
		}	
		if(detectLine >= 5)
		{	
			estado = 2;
			while(readLineSensors(0) > 5)
			{		
				Rodar_Sobre_Si();
			}
			
		}
	}
}
//####################################################################################################
/*esta funcao tem de ser alterada pois falta alenhar o servo para quando ele estiver a -15 ou 15 fazer 90 graus com a posicao zero */
/*ver se era por causa do while que isto nao alilhava*/
	void Ver_Farol()
{	
	
	stop_Motors();
	int position = -15, count=0;
	static unsigned int lado=0;
	readAnalogSensors();
	do{
		setServoPos(position);
		position++;
		
	
		//printf("farol: %d\n", readBeaconSens() );
		
		if(position== (7)){
			position= -15;
			count++;
			if(lado == 0 && count <= 3  ){
				rotateRel_naive(normalizeAngle(-M_PI/2));
				printf("lado 0\n");
				if(count == 3){
					lado++;
				}		

			}else if(lado == 1 && count <= 3  ) {
				rotateRel_naive(normalizeAngle(M_PI/2));
				printf("lado 1\n");
				if(count == 3){
					lado =0;
				}
			}
		}
		delay(500);

	}while(readBeaconSens() ==0  && count < 4);


	//printf("%d \n", position);
	if(position < 0 && count < 4){
		//printf("servo position < 0\n");	
		

			rotateRel_naive(normalizeAngle((position *M_PI/2) / (15) ));
			
		
	}else if(position > 0 && count < 4){
		//printf("servo position > 0\n");S
	
			rotateRel_naive( normalizeAngle ((position *M_PI/2) / -15));
			
		
	}
	andar_frente();
	setServoPos(0);
}
//###########################################3
void rotateRel_naive(double deltaAngle)
{
	double x, y, t;
	double targetAngle;
	double error;
	int cmdVel;

	getRobotPos(&x, &y, &t);
	targetAngle = normalizeAngle(t + deltaAngle);
	error = normalizeAngle(targetAngle - t);
	if(error < 0)
		cmdVel = -30;
	else
		cmdVel = 30;

	setVel2(-cmdVel, cmdVel);

	do
	{
		getRobotPos(&x, &y, &t);
		error = normalizeAngle(targetAngle - t);
	} while (fabs(error) > 0.01 && (error * cmdVel) > 0);
	setVel2(0, 0);

}
//#####################################################################################
/* TimeOut e a funçao que desliga o robot  do fim de 3 min*/
/* com os 160ms para fazer 3min=180000ms sa presisos 1125ciclos 
*/
void TimeOut(){

	//int tempo=readCoreTimer();
	
		if(tick160ms==1){
			tick160ms==0;
			ciclos++;
		}
		//printf("read %d \n", ciclos);
		if(ciclos>=1125){		//falta ver o valor certo, mas ja funciona(so no dia da competicao)
			Fim();
		}
			//reset a flag
		
}
/* esta funçao serve para mostrar que robô dê a sua prova por concluída, tendo ou não atingido objetivo.
*Para isso os led devem permanecer intermitente, com uma frequência compreendida entre 1 e 5 Hz*/
void Fim(){

	Stop_robot();
	
	//esta a frequencia 1hz
	while(TRUE){
		printStr("TimeOut\n");
		leds(0xF);
		wait(5);
		leds(0x0);
		wait(5);
	}
}
