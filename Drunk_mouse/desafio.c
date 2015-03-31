#include "mr32.h"

// VARIABLES
#define LIMIAR 450
#define Velocidade 70 // Testar amanha!!

#define TRUE 1
#define FALSE 0



volatile int sensor_dir,sensor_esq,sensor_frente; 
volatile int estado=0;     /* 0 = parado; 1 = a trabalhar */
volatile int linha = 0;
volatile int farolsen = 0;
volatile int reset = 0;
/*int a_s_dir[10]= {0, 0, 0, 0, 0,0, 0,0, 0, 0};
int a_s_esq[10]= {0, 0, 0, 0, 0,0, 0,0, 0, 0};
int a_s_frente[10]= {0, 0, 0, 0, 0,0, 0,0, 0, 0};*/   /*olaTeste2*/

int ciclos=0;	//tempo em funcionamento



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
void TimeOut(void);
/********************************************/
int main (void)
{
	//iniciçao da pic
  	initPIC32 ();
  	closedLoopControl( true );
  	setVel2(0, 0);

  	T4CONbits.TCKPS=5;
PR4=(PBCLK/32/3)-1;//fout 20MHz/(32*(62499+1))*10Hz,freq a 1 hz
TMR4=0;//Reset timer T2
T4CONbits.TON=1;
//Interrupts
IFS0bits.T4IF=0;//reset do timer
IPC4bits.T4IP=2;//priority
IEC0bits.T4IE=1;//enable timer3
EnableInterrupts();

	printStr(__FILE__); // para saber o nome do ficheiro que esta a correr no robot
	printf("\r    battery: %d ", batteryVoltage());
	printStr("\n");
	

//desligar os leds para o inicio
	leds(0b1111);	//leds off


  #if 0

	while(1)
	{
		while(!tick40ms);
		tick40ms=0;
		readAnalogSensors();
		


	}


  #endif



  while (TRUE)
    {
	
	
	TimeOut();		// timeOut => tb devia ir para uma inturrupcao

	//Sensor(); 	 //leitura os sensores tem de se por numa interropecao

	


	//botao de estado 

	if(startButton() == 1) // Botao start(preto) primido
	{
		estado = 1;
		resetCoreTimer();
		leds(0x1);
	
	}
	else if(stopButton() == 1) //Botao stop(vermelho) primido
	{
		estado = 0;
		leds(0x0);
	}

 	//funcoa de funcionameneto
	if(estado == 1) // robot fica em presesamento
	{
		Ver_Farol();
		printf("D: %d   F: %d   E: %d     LINHA: %d    Farol: %d \r",	sensor_dir,sensor_frente,sensor_esq,linha ,farolsen );
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



//########################################################################################################################################################################
/* conficoraçao dos metores*/

void Stop_robot()  //serve para quando se carrega no botao de desligar
{
	setVel2(0,0);		// desliga os metores
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




//########################################################################################################################################################################
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
		setVel2(0,0);
		Rodar_Sobre_Si();
		wait(1);

	
	}
	

}

//##################################################################################################################################################################3
void Sensor()
{
	//int sum_dir=0, sum_esq=0, sum_frente=0;
	printf("Sensor\n");
	//para garantir que funcionao
	disableObstSens();
	//disableLineSens();
	//enableLineSens();
	enableObstSens();
		
	readAnalogSensors();
	

	/*int i=0;
	for( i=0; i<9; i++){
		a_s_dir[i]=a_s_dir[i+1];
		a_s_esq[i]=a_s_esq[i+1];
		a_s_frente[i]=a_s_frente[i+1];
	}
 
	
	a_s_dir[9] = obstacleSensor(OBST_SENSOR_RIGHT);
	a_s_esq[9] = obstacleSensor(OBST_SENSOR_LEFT);
	a_s_frente[9]= obstacleSensor(OBST_SENSOR_FRONT);
	

	*/
	Chegada_Farol(); 		//verificao da chegada ao farol substitui => linha = lineSensor(LINE_SENSOR_CENTER);


#if 0
	if((ciclos%10 )== 0){			//verificar isto pois nao sei se ele ve o farol so de 5 em 5 s
		Ver_Farol();	
	}
#else
	Ver_Farol();
#endif

	
	/*// media dos sensores

	for( i=0; i<9; i++){
		sum_dir=+ a_s_dir[i];
		sum_esq=+ a_s_esq[i];
		sum_frente=+a_s_frente[i];
	}



	sensor_esq = sum_dir/10;
	sensor_dir = sum_esq/10;
	sensor_frente= sum_dir/10;*/
	sensor_dir = obstacleSensor(OBST_SENSOR_RIGHT);
	sensor_esq = obstacleSensor(OBST_SENSOR_LEFT);
	sensor_frente= obstacleSensor(OBST_SENSOR_FRONT);


	//printf("D: %d   F: %d   E: %d     LINHA: %d    Farol: %d \r", obstacleSensor(0), obstacleSensor(1), obstacleSensor(2), linha, farolsen );

}

//############################################################################################################################################################3
void Chegada_Farol ()
{
	
	if (readLineSensors(0) == 1 )//||readLineSensors(1) == 1 || readLineSensors(2) == 1 || readLineSensors(3) == 1)
	{	
		wait(2); //para se ver melhor depois
		Stop_robot();
		estado = 0;
		printf("Cheguei ao farol  CARALHO!!!!!!!!!!!!!!!!!\n\n");
	}
	
}

//################3
/*esta funcao tem de ser alterada pois ele tem de ver se encotra o farol a 360º */
void Ver_Farol()
{	
	setServoPos(0);
	readAnalogSensors();
	if( (farolsen = readBeaconSens())==1)
	{
		andar_frente();
	}
	
}



//############################################################################################################################################################
/* TimeOut e a funçao que desliga o robot  do fim de 3 min*/
/*a flag muda para 1 a cada 160ms por isso para o robot 3 min sao 180000 logo sao 1125 ciclos
*ciclos = ms_activo/160
*/
void TimeOut(){

	//int tempo=readCoreTimer();
	
	if (tick160ms==1)// 
		 {
			ciclos++;
			//printf("read %d \n", ciclos);
			if(ciclos>=1125){		//falta ver o valor certo, mas ja funciona
				Fim();
			}
		tick160ms=0;	//reset a flag
		}
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
//######################################################################################################################################################################33
void _int_(_TIMER_4_VECTOR) isr_t22(void)
{

	Sensor();
	printf("puta Cheguei\n");
	
	IFS0bits.T4IF = 0;
}