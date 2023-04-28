
int in_ADC0, in_ADC1;  //variables para ADC 0 y 1
int POT0, POT1, POT2, out_DAC0, out_DAC1; //Variables para los 3 potenciometros, variables para DACS
const int LED = 3;  //variable LED
const int FOOTSWITCH = 7; //variable FOOTSWITCH
const int SWITCH = 2; //variable Swtich 
 
#define MAX_DELAY 2000 //definir el delay maximo
uint16_t sDelayBuffer0[MAX_DELAY-1]; //crear variable unsigned_short con valor max de 0xFFFF menos uno (octava abajo)
uint16_t sDelayBuffer1[MAX_DELAY-1]; //crear variable unsigned_short con valor max de 0xFFFF menos uno (octava abajo) 
unsigned int write_pt=0;
unsigned int read_pt_A=0, read_pt_B= MAX_DELAY/2;
unsigned int Delay_Depth, increment, divider=0, buffer0, buffer1;
 
void setup()
{
   //Encender Timer
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC4);
 
  //iniciar la onda de muestreo
  TC_Configure(TC1,1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
  TC_SetRC(TC1, 1, 238); // inicia el muestreo a  44.1 Khz en las interrupciones
  TC_Start(TC1, 1);
 
  //activar interrupciones de timer en el timer
  TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;
 
  //Activar la interrupcion en el vector anidado de la interrupcion
  //TC4_IRQn donde 4 es el numero de timer * canales de timer (3 en total) + el numero de canal 
  //(=(1*3)+1) for timer1 channel1 
  NVIC_EnableIRQ(TC4_IRQn);
  
  //Configuracion ADC
  ADC->ADC_MR |= 0x80;   // DAC en modo libre
  ADC->ADC_CR=2;         // Iniciar conversión de ADCs
  ADC->ADC_CHER=0x1CC0;  // Activar ADCs mediante regristro
 
  //Configuracion DAC
  analogWrite(DAC0,0);  // Activar Dac 0
  analogWrite(DAC1,0);  // Activar Dac 1
 
  //Configuración de usuario
  pinMode(LED, OUTPUT);  
  pinMode(FOOTSWITCH, INPUT_PULLUP);      
  pinMode(SWITCH, INPUT_PULLUP);  
}
 
void loop()
{
  //Encender el LED al activar el efecto!
  if (digitalRead(FOOTSWITCH)) digitalWrite(LED, HIGH); 
    else  digitalWrite(LED, LOW);
 
  //Leer ADCs
  while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// Esperar la conversión de los ADC
  in_ADC0=ADC->ADC_CDR[7];               // leer datos ADC0
  in_ADC1=ADC->ADC_CDR[6];               // leer datos ADC1  
  POT0=ADC->ADC_CDR[10];                 // leer datos ADC8        
  POT1=ADC->ADC_CDR[11];                 // leer datos ADC9   
  POT2=ADC->ADC_CDR[12];                 // leer datos ADC10     
}
 
void TC4_Handler() //Interrupt at 44.1KHz rate (every 22.6us)
{
   //Limpiar estado para activar de nuevo la interrupcion
  TC_GetStatus(TC1, 1);
 
  //Almacenar lecturas actuales
  sDelayBuffer0[write_pt] = in_ADC0;
  sDelayBuffer1[write_pt] = in_ADC1;
 
  //Ajustar delay dependiendo en potenciometro 0
  Delay_Depth = MAX_DELAY-1;
 
  //Aumentar y reiniciar el contador
  write_pt++;
  if(write_pt >= Delay_Depth) write_pt = 0; 
 
  out_DAC0 = ((sDelayBuffer0[read_pt_A]));
  out_DAC1 = ((sDelayBuffer1[read_pt_B]));
 
  if (POT0>2700)  //control de "octava arriba"
  { 
    read_pt_A = read_pt_A + 2;
    read_pt_B = read_pt_B + 2;
  }
 else if (POT0>1350) //control de "octava actual"
  {
    read_pt_A = read_pt_A + 1;
    read_pt_B = read_pt_B + 1;
  }
 else
 {
   divider++;
   if (divider>=2) //octava abajo
   {
      read_pt_A = read_pt_A + 1;
      read_pt_B = read_pt_B + 1;
      divider=0;
    }
  }
 
  if(read_pt_A >= Delay_Depth) read_pt_A = 0; 
  if(read_pt_B >= Delay_Depth) read_pt_B = 0; 
 
  //Control de volumen potenciometro 2
  out_DAC0=map(out_DAC0,0,4095,1,POT2);
  out_DAC1=map(out_DAC1,0,4095,1,POT2);
 
  //Escribir en los DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);       
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);
  dacc_set_channel_selection(DACC_INTERFACE, 1);       
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);
}
