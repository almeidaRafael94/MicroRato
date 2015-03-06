#include "mr32.h"

// VARIABLES
#define LIMIAR 300
#define Velocidade 100 // Testar amanha!!


//int ledID = 0;    /* Led 1 de funcionamento*/
//int ledID1 = 1;   /* Led 2 de andar em frante */
//int ledID2 = 2;   /* Led 3 de rotacao em si proprio */
int ledID3 = 3;   /* Led 4 quando tem de virar para um dos lados */
int sensor_dir,sensor_esq,sensor_frente; 
int estado=0;     /* 0 = parado; 1 = a trabalhar */
int linha = 0;
int farolsen = 0;
int reset = 0;
int a_s_dir[3]= {0, 0, 0};
int a_s_esq[3]= {0, 0, 0};
int a_s_frente[3]= {0, 0, 0};


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
void Sensor(void);

/********************************************/
int main (void)
{

  initPIC32 ();

	printStr(__FILE__); // para saber o nome do ficheiro que esta a correr no robot
	printf("\r    battery: %d ", battery());
	printStr("\n");
	

  while (TRUE)
    {
	
	
	Fim();
	//leitura os sensores
	if ( readCoreTimer() > 2000000*180)
   	 {
		stopMotors();
		setLed(1);
	

	}

	Sensor();
	

	
	printf("D: %d   F: %d   E: %d     LINHA: %d    Farol: %d \r", obstacleSensor(0), obstacleSensor(1), obstacleSensor(2), linha, farolsen );

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

void Sensor()
{
	int sum_dir=0, sum_esq=0, sum_frente=0;

	
	/*int i=0;
	for(i=0; i<3, i++){*/
		a_s_dir[1]=a_s_dir[2];
		a_s_esq[1]=a_s_esq[2];
		a_s_frente[1]=a_s_frente[2];
		
	


	disableObstSens();
	disableLineSens();
	enableLineSens();
	enableObstSens();
		
	readSensors();  
	
	a_s_dir[3] = obstacleSensor(OBST_SENSOR_RIGHT);
	a_s_esq[3] = obstacleSensor(OBST_SENSOR_LEFT);
	a_s_frente[3]= obstacleSensor(OBST_SENSOR_FRONT);
	linha = lineSensor(LINE_SENSOR_CENTER);
	farolsen = readBeaconSens();

	
	sum_dir= a_s_dir[1] + a_s_dir[2] + a_s_dir[3]; 
	sum_esq= a_s_esq[1] + a_s_esq[2] + a_s_esq[3]; 
	sum_frente= a_s_frente[1] + a_s_frente[2] + a_s_frente[3]; 


	sensor_esq = sum_dir/3;
	sensor_dir = sum_esq/3;
	sensor_frente= sum_dir/3;




}

