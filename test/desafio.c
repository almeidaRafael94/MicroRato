#include "mr32.h"

// VARIABLES
#define LIMIAR 300
#define Velocidade 100 // Testar amanha!!





int sensor_dir,sensor_esq,sensor_frente; 
int estado=0;     /* 0 = parado; 1 = a trabalhar */
int linha = 0;
int farolsen = 0;
int reset = 0;
int a_s_dir[3]= {0, 0, 0};
int a_s_esq[3]= {0, 0, 0};
int a_s_frente[3]= {0, 0, 0};
int seg=0;	

/********************************************/
// FUNCTIONS

void Stop_robot(void);
void Ajusta_Dir(void);
void Ajusta_Esq(void);
void Vira_Dir (void);
void Rodar_Sobre_Si (void);
void andar_frente (void);
void Vira_esq (void);
void Chegada_Farol (void);
void Ver_Farol(void);
void ANDAR1(void);
void ANDAR2(void);
void Sensor(void);
void Fim(void);  
/********************************************/
int main (void)
{

  initPIC32 ();

	printStr(__FILE__); // para saber o nome do ficheiro que esta a correr no robot
	printf("\r    battery: %d ", battery());
	printStr("\n");
	

//desligar os leds para o inicio
	resetLed(0);
	resetLed(1);
	resetLed(2);
	resetLed(3);

  while (TRUE)
    {
	
	
	
	
	if ( readCoreTimer() >= 20000000)
   	 {
		seg++;
		if(seg>=180){
			Fim();
		}
	
	}

	Sensor(); 	 //leitura os sensores
	printf("D: %d   F: %d   E: %d     LINHA: %d    Farol: %d \r", obstacleSensor(0), obstacleSensor(1), obstacleSensor(2), linha, farolsen );


	//botao de estado 

	if(startButton() == 1) // Botao start(preto) primido
	{
		estado = 1;
		resetCoreTimer();
	}
	else if(stopButton() == 1) //Botao stop(vermelho) primido
	{
		estado = 0;
	}

 	//funcoa de funcionameneto
	if(estado == 1) // robot fica em presesamento
	{
		Ver_Farol();		
		ANDAR2();
		
	}
	//quando e primido o botao stop
	else if(stopButton() == 1)		// deslica o funcionamento, nenhum led activo
	{
		
		Stop_robot();
	
	}

    }
  return (0);
}
//################3
void Stop_robot()  //serve para quando se carrega no botao de desligar
{
	stopMotors();		// desliga os metores
	estado = 0;
}
//################3
void Ajusta_Esq ()
{
	setVel2 (Velocidade-20,Velocidade);
}
//################3
void Ajusta_Dir ()
{
	setVel2 (Velocidade,Velocidade-20);
}
//################3
void Vira_Dir ()
{
	setVel2 (Velocidade-40, Velocidade); // ALTERAR ISTO
}
//################3
void Vira_esq ()
{
	setVel2 (Velocidade, Velocidade-40); // ALTERAR ISTO
}
//################3
void andar_frente ()
{
	setVel2 (Velocidade, Velocidade); // ALTERAR ISTO
}
//################3
void Rodar_Sobre_Si ()
{
	setVel2 (Velocidade, -Velocidade);
}
//################3
void Chegada_Farol ()
{
	
	if (lineSensor(LINE_SENSOR_LEFT1) == 1 ||lineSensor(LINE_SENSOR_CENTER) == 1 || lineSensor(LINE_SENSOR_RIGHT1) == 1 || lineSensor(LINE_SENSOR_RIGHT2) == 1)
	{	
		wait(2); //para se ver melhor depois
		Stop_robot();
		estado = 0;
		printf("Cheguei ao farol  CARALHO!!!!!!!!!!!!!!!!!\n\n");
	}
	
}
//################3
void Ver_Farol()
{	
	setServoPos(0);
	readSensors(); 
	if(readBeaconSens()==1)
	{
		andar_frente();
	}
	
}
//################3
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
//################3
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

//################3
void Sensor()
{
	int sum_dir=0, sum_esq=0, sum_frente=0;

	
	int i=0;
	for( i=0; i<2; i++){
		a_s_dir[i]=a_s_dir[i+1];
		a_s_esq[i]=a_s_esq[i+1];
		a_s_frente[i]=a_s_frente[i+1];
		
	
	}

	disableObstSens();
	disableLineSens();
	enableLineSens();
	enableObstSens();
		
	readSensors();  
	
	a_s_dir[2] = obstacleSensor(OBST_SENSOR_RIGHT);
	a_s_esq[2] = obstacleSensor(OBST_SENSOR_LEFT);
	a_s_frente[2]= obstacleSensor(OBST_SENSOR_FRONT);
	

	linha = lineSensor(LINE_SENSOR_CENTER);
	Chegada_Farol(); //verificao da chegada ao farol
	farolsen = readBeaconSens();

	
	sum_dir= a_s_dir[0] + a_s_dir[1] + a_s_dir[2]; 
	sum_esq= a_s_esq[0] + a_s_esq[1] + a_s_esq[2]; 
	sum_frente= a_s_frente[0] + a_s_frente[1] + a_s_frente[1]; 


	sensor_esq = sum_dir/3;
	sensor_dir = sum_esq/3;
	sensor_frente= sum_dir/3;




}
//######################################################################################################################################################33
/* esta funçao serve para mostrar que robô dê a sua prova por concluída, tendo ou não atingido objetivo.
*Para isso os led devem permanecer intermitente, com uma frequência compreendida entre 1 e 5 Hz*/
void Fim(){

	stopMotors();
	//esta a frequencia 1hz
	while(TRUE){
		setLed(0);
		setLed(1);
		setLed(2);
		setLed(3);
		wait(1000);
		resetLed(0);
		resetLed(1);
		resetLed(2);
		resetLed(3);
		wait(1000);
	}

}
