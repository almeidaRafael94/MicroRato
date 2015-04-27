#include "mr32.h"

// VARIABLES
#define LIMIAR 300
#define Velocidade 50 // TESTar amanha!!


//int ledID = 0;    /* Led 1 de funcionamento*/
//int ledID1 = 1;   /* Led 2 de andar em frante */
//int ledID2 = 2;   /* Led 3 de rotacao em si proprio */
int ledID3 = 3;   /* Led 4 quando tem de virar para um dos lados */
int sensor_dir,sensor_esq,sensor_frente; 
int estado=0;     /* 0 = parado; 1 = a trabalhar */
int val = 0;
int linha = 0;
int farolsen = 0;
int reset = 0;


/********************************************/
// FUNCTIONS

void Stop_robot(void);
void Ajusta_Dir(void);
void Ajusta_Esq(void);
void Vira_Dir (void);
void Rodar_Sobre_Si (void);
void andar_frente (void);
void Vira_esq (void);
void Fim (void);
void farol(void);
void ANDAR1(void);
void ANDAR2(void);
void ANDAR3(void);

/********************************************/
int main (void)
{

  initPIC32 ();

	printStr(__FILE__); // para saber o nome do ficheiro que esta a correr no robot
	printf("    battery: %d ", battery());
	printStr("\n");
	

  while (TRUE)
    {
	
	disableObstSens();
	disableLineSens();
	enableLineSens();
	enableObstSens();
		
	Fim();
	//leitura os sensores
	if ( readCoreTimer() > 2000000*180)
   	 {
		stopMotors();
		setLed(1);
	

	}
	readSensors();  
	
	sensor_dir = obstacleSensor(OBST_SENSOR_RIGHT);
	sensor_esq = obstacleSensor(OBST_SENSOR_LEFT);
	sensor_frente = obstacleSensor(OBST_SENSOR_FRONT);
	linha = lineSensor(LINE_SENSOR_CENTER);
	farolsen = readBeaconSens();

	
	printf("D: %d   F: %d   E: %d     LINHA: %d    Farol: %d \n", obstacleSensor(0), obstacleSensor(1), obstacleSensor(2), linha, farolsen );

	//botao de estado 

	if(startButton() == 1) // Botao start(preto) primido
	{
		estado = 1;
	}
	else if(stopButton() == 1) //Botao stop(vermelho) primido
	{
		estado = 0;
	}

 	//funcoa de funcionameneto
	if(estado == 1) // robot fica em presesamento
	{
		//setLed(ledID);// led que indica que o robot esta a funcionar
		if(reset ==0 ){
		resetCoreTimer();
		reset = 1;
		}
		resetLed(ledID3);
		farol();		
			
		//setVel2(Velocidade,Velocidade); // inicia movimento

		ANDAR2();

			
/*************************************************************/
	}
	//quando e primido o botao stop
	else if(stopButton() == 1)		// deslica o funcionamento, nenhum led activo
	{
		
		Stop_robot();
	
	}

    }
  return (0);
}

void Stop_robot()
{
	//resetLed(ledID);	// desliga o led de funcionamento
	stopMotors();		// desliga os metores
	estado = 0;
}

void Ajusta_Esq ()
{
	setVel2 (Velocidade-20,Velocidade);
}

void Ajusta_Dir ()
{
	setVel2 (Velocidade,Velocidade-20);
}

void Vira_Dir ()
{
	setVel2 (Velocidade-40, Velocidade); // ALTERAR ISTO
}
void Vira_esq ()
{
	setVel2 (Velocidade, Velocidade-40); // ALTERAR ISTO
}
void andar_frente ()
{
	setVel2 (Velocidade, Velocidade); // ALTERAR ISTO
}
void Rodar_Sobre_Si ()
{
	setVel2 (Velocidade, -Velocidade);
}

void Fim ()
{
	
	if (lineSensor(LINE_SENSOR_LEFT1) == 1 ||lineSensor(LINE_SENSOR_CENTER) == 1 || lineSensor(LINE_SENSOR_RIGHT1) == 1 || lineSensor(LINE_SENSOR_RIGHT2) == 1)
	{	
		wait(2); //para se ver melhor depois
		Stop_robot();
		setLed(ledID3);
		estado = 0;
		printf("Chegei ao farol  CARALHO!!!!!!!!!!!!!!!!!\n\n");
	}
	
}
void farol()
{	
	setServoPos(0);
	readSensors(); 
	if(readBeaconSens()==1)
	{
		andar_frente();
	}
	
}

void ANDAR1 ()
{
		if(sensor_dir < LIMIAR && sensor_frente < LIMIAR && sensor_esq < LIMIAR )
		{	

			andar_frente();
		}
		 if(sensor_dir > LIMIAR && sensor_frente > LIMIAR && sensor_esq > LIMIAR )
		{
			Rodar_Sobre_Si();
		}
		 if(sensor_dir < LIMIAR && sensor_frente > LIMIAR && sensor_esq > LIMIAR )
		{
			Vira_esq();
			//Ajusta_Esq();
		}
		 if(sensor_dir > LIMIAR && sensor_frente > LIMIAR && sensor_esq < LIMIAR )
		{
			Vira_Dir();
			//Ajusta_Dir();
		}

}

void ANDAR2 ()
{
	if(sensor_dir < LIMIAR)
	{
		Vira_Dir();
		
	}
	if(sensor_esq > LIMIAR)
	{
		Vira_esq();
		
	}
	if(sensor_frente < LIMIAR && sensor_dir < LIMIAR&& sensor_esq < LIMIAR)
	{	
		andar_frente();
	}
	if(sensor_frente > LIMIAR && sensor_dir > LIMIAR && sensor_esq > LIMIAR)
	{	
		stopMotors();
		
		Rodar_Sobre_Si();
		wait(1);

	
	}
	

}


void ANDAR3 ()
{
	if(sensor_dir < LIMIAR)
	{
		Vira_Dir();
		
	}
	if(sensor_esq > LIMIAR)
	{
		Vira_esq();
		
	}
}

