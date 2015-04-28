#include "mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// VARIABLES
#define LIMIAR 450
#define Velocidade 70
#define Rodar 63

#define TRUE 1
#define FALSE 0

#define sizeArray 4500


volatile int sensor_dir,sensor_esq,sensor_frente; 
volatile int estado=0;     /* 0 = parado; 1 = Run_Beacon; 2 = ver_farol; 3= Go_home; 4 = End */
volatile int linha = 0;
volatile int farolsen = 0;
volatile int reset = 0;
int ciclos=0;				//tempo em funcionamento
int countRodarFarol=0; 				//count para o robot ver o farol
int countCiclos = 100;		//ciclos para ir ver o farol
static unsigned lado=0;
//array de posicao
double xx [sizeArray];
double yy [sizeArray];
double tt [sizeArray];
int indexA=0;
int test=0;
int stateRondomDecision = 0;
int stop = 0;
int indexInvert = 0;
/********************************************/
// FUNCTIONS

void Stop_robot(void);		//deliga os motores e os sensores de obstaculos
void stop_Motors(void); 	//para os motores 
//funcoes para andar
void Ajusta_Dir(void);
void Ajusta_Esq(void);
void Vira_Dir (void);
void Rodar_Sobre_Si_Esq (void);
void Rodar_Sobre_Si_Dir (void);
void Rodar_Sobre_Si (void);
void andar_frente (void);
void Vira_esq (void);
void Run_Beacon(void);
void ANDAR2 ();
void rotateRel_naive(double deltaAngle);	//virar partir de um anglo em radiano
//verificacao do farol
void Chegada_Farol (void);
void Ver_Farol(int);
//tempo de funcionamento
void Fim(void);  
void TimeOut(void);

//funcoes para voltar para a partida
void return_Home();
int storePosition(void);
int arraySize(void);
/********************************************/
int main (void)
{
	indexInvert = indexA;
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
		//printf("%d\n", ciclos );
		while(!tick40ms);
		tick40ms=0;
		readAnalogSensors();

		//state buttons

		if(startButton() == 1) 				// Botao start(preto) primido
		{
			estado = 1;
			enableObstSens();
			leds(0x0);						//leds off
			countCiclos = 100;
		}

		else if(stopButton() == 1) 			//Botao stop(vermelho) primido
		{
			estado = 0;
			disableObstSens();
		}
//estaddos
			//printf("%d\n", estado);
		switch(estado){
			case 0:				
				setVel2(0,0); //stop_Motors();
				disableObstSens();
				break;

			case 1:
			/*Ida para o farol*/
				
				TimeOut();					// timeOut => tb devia ir para uma inturrupcao
				
				storePosition();  
				Chegada_Farol();
				if(countCiclos++ >= 40)
				{
					estado = 2; 
					countCiclos = 0;
				}
				Run_Beacon();
				break;

			case 2:
			/*procura do farol*/
				TimeOut();						// timeOut => tb devia ir para uma inturrupcao
				Ver_Farol(lado);
				if(countRodarFarol++ >= Rodar){
					estado = 1;
					countRodarFarol = 0;
					if(lado == 0){
						lado =1;
					} else if(lado == 1) {
						lado=0;
					}
				}
				break;

			case 3:
				/*estado atciva depois de chegar ao farol. 
				volta para o ponto de partida*/
				leds(0x1);
				TimeOut();
				return_Home();
				break;

			case 4:
			/*timeOut: desliga os motores e os leds piscam ao fim de 3 min*/
				Fim();
				break;
			case 5:
			/*Chegada a casa*/
				leds(0x6);
				TimeOut();
				stop_Motors();
				break;
			default:
			/*estado de seguranca, em caso do robot se passe*/
				estado=2;
				leds(0x7);
				break;

			}

		if(stopButton() == 1)		// deslica o funcionamento, nenhum led activo
		{
			stop = 1;
			if(stop == 1)
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
	disableObstSens();
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

	if((obstacleSensor(OBST_SENSOR_FRONT) > LIMIAR)  &&  // parede LEFT
	    (obstacleSensor(OBST_SENSOR_LEFT) > LIMIAR) && 
	    (obstacleSensor(OBST_SENSOR_RIGHT) < LIMIAR))
	{
		return 0;
	}
	else if((obstacleSensor(OBST_SENSOR_FRONT) > LIMIAR) &&  // parede 	Right
			(obstacleSensor(OBST_SENSOR_RIGHT) > LIMIAR) && 
	    	(obstacleSensor(OBST_SENSOR_LEFT) < LIMIAR))
	{
		return 1;
		
	}
	else
	{
		if(stateRondomDecision == 0)
		{	
			stateRondomDecision = 1;
			return 0;
		}
		else
		{	
			stateRondomDecision = 0;
			return 1;
		}
	}
}
void stop_Motors()
{
	setVel2(0,0);
}

//#############################################################################
void Run_Beacon ()
{
	//disableObstSens();
	
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
			estado = 3;
		}
	}
}
//####################################################################################################
/*esta funcao tem de ser alterada pois falta alenhar o servo para quando ele estiver a -15 ou 15 fazer 90 graus com a posicao zero */
/*ver se era por causa do while que isto nao alilhava*/
	void Ver_Farol(int virar)
{	
	
	setServoPos(0);
	stop_Motors();

	if(virar == 0){
		rotateRel_naive(normalizeAngle(0.1));
	}else if(virar == 1){
		rotateRel_naive(normalizeAngle(-0.1));
	}

	

	if(readBeaconSens() ==1){
		estado =1;
	
		//wait(2);	
	}
	//printf("farol= %d; count=%d ; lado: %d \n", readBeaconSens(), countRodarFarol, lado);

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
/* com os 40ms para fazer 3min=180000ms sa presisos 4500ciclos 
*/
void TimeOut(){

	//int tempo=readCoreTimer();
	//printf("%d\n", ciclos );
			ciclos++;
		
		//printf("read %d \n", ciclos);
		if(ciclos>=500){		//falta ver o valor certo, mas ja funciona(so no dia da competicao)
			estado =4;
		}
			//reset a flag-0.007673

		
}
/* esta funçao serve para mostrar que robô dê a sua prova por concluída, tendo ou não atingido objetivo.
*Para isso os led devem permanecer intermitente, com uma frequência compreendida entre 1 e 5 Hz*/
void Fim(){

	stop_Motors();
	
	//esta a frequencia 1hz
	int x = 0;
	for(x = 0; x < 300; x++)
	{
		printf("%f, %f, %f\n", xx[x],yy[x], tt[x]);
	}
	while(TRUE){
		printStr("TimeOut\n");
		leds(0xF);
		wait(5);
		leds(0x0);
		wait(5);
	}
}
//################################################################################################
void return_Home()
{
	//Run_Beacon();

	double x, y, t;

			//Run_Beacon();

			getRobotPos(&x, &y, &t);
			//printf("x:%f  y:%f  TETA:%f\n", x, y, t); // print Position
			if (t != tt[0])
			{
				if(test++==20){
					rotateRel_naive(( PI-normalizeAngle(t)));//t[indexInvert--]));
					test = 0;
				//	printf("%s\n", "AJUSTA ANGULO PARA A BASE" );
				}
			
				ANDAR2();
				
			}
			
			if(readLineSensors(0) >= 5)
			{	
				rotateRel_naive(normalizeAngle(PI));
			}

			if ((x <= abs(xx[0]+20)) && (y <= abs(yy[0]+20)))
			{
				estado = 5; // PARAR, chegei a casa
			}
}

int storePosition(void)
{
	double x, y, t;
	//while(!tick40ms);
	//tick40ms = 0;

	if(estado != 3)	// se estado = 3 não guarda as posições
	{
		if (indexA <= arraySize())
		{
			getRobotPos(&x, &y, &t); //printf("x:%f  y:%f  TETA:%f\n", x, y, t); // print Position

			xx[indexA] = x;
			yy[indexA] = y;
			tt[indexA] = t; // Store values
			//printf("x:%f  y:%f  TETA:%f\n", x, y, t); // print Position
			//printf("indexA = %d\n", indexA);
			//printf("Size: %d\n", arraySize());
			indexA++;
			return 0;	
		}
		else
		{
			//printf("Array cheio!! \n");
			//printArrayPos();
			//arrayCheio = 1;
			return 1;
		}
	}

}

int arraySize(void)
{
	return (sizeof(xx)/sizeof(double*)/2);
}


//#####
void ANDAR2 ()
{
	if(obstacleSensor(OBST_SENSOR_RIGHT) < LIMIAR)
	{
		Vira_Dir();
		
	}
	if(obstacleSensor(OBST_SENSOR_LEFT) > LIMIAR)
	{
		Vira_esq();
		
	}
	if(obstacleSensor(OBST_SENSOR_FRONT) < LIMIAR && obstacleSensor(OBST_SENSOR_RIGHT) < LIMIAR&& obstacleSensor(OBST_SENSOR_LEFT) < LIMIAR)
	{	
		andar_frente();
	}
	if(obstacleSensor(OBST_SENSOR_FRONT) > LIMIAR && obstacleSensor(OBST_SENSOR_RIGHT) > LIMIAR && obstacleSensor(OBST_SENSOR_LEFT) > LIMIAR)
	{	
		setVel2(0,0);
		
		Rodar_Sobre_Si();
		wait(1);

	
	}
	

}