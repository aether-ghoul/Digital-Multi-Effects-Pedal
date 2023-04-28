    #define ModBluetooth Serial1 //Variable para establecer los pines para la comunicacion seria. (serial1= 18 y 19)
    
   int in_ADC0, in_ADC1;  //variables para ADC 0 y 1
   int POT0, POT1, POT2, out_DAC0, out_DAC1; //Variables para los 3 potenciometros, variables para DACS
   const int LED = 3;  //variable LED
   const int FOOTSWITCH = 7; //variable FOOTSWITCH
   const int SWITCH = 2; //variable Swtich 
   const int POWER = 8;
   int Dato=0;
   int potenciometro0;
   int potenciometro1;
   int potenciometro2;
   int fijar;

   
   
   
    //VARIABLES PARA EFECTO DISTORSION
   int lim_sup, lim_inf; //Variable limite superior e inferior

     #define MAX_DELAY_A 30000  //definir el delay maximo
     uint16_t muestraDelay[MAX_DELAY_A]; //crear variable unsigned_short con valor max de 0xFFFF, 16 bits, para las muestras de 12 bits
     unsigned int Profundidad_Delay, Contador_Delay = 0; //crear variable unsigned int para la profundidad de delay y contador
     #define MAX_DELAY_B 500
    #define MIN_DELAY 200
uint16_t sDelayBuffer0[MAX_DELAY_B+500];

unsigned int DelayCounter = 0;

unsigned int count_up=1;
int p;


   #define MAX_DELAY 2300 //definir el delay maximo
   uint16_t muestraDelay0[MAX_DELAY]; //crear variable unsigned_short con valor max de 0xFFFF menos uno (octava abajo)
   uint16_t muestraDelay1[MAX_DELAY]; //crear variable unsigned_short con valor max de 0xFFFF menos uno (octava abajo) 
   unsigned int write_pt=0;
   unsigned int read_pt_A=0, read_pt_B= MAX_DELAY/2;
    unsigned int Delay_Depth, increment, divider=0, buffer0, buffer1;

   int sample, accumulator, count, LFO;
 
// Crear tabla de muestreo
#define no_samples 11000

uint16_t nSineTable[no_samples];//Guardando muestras de 12 bits en variable de 16 bits
 
// crear muestras individuales para nuestra tabla de muestreo
void createSineTable()
{
  for(uint32_t nIndex=0; nIndex<no_samples; nIndex++)
  {
    // Ajustar a 12 bits solamente
    nSineTable[nIndex] = (uint16_t)  (((1+sin(((2.0*PI)/no_samples)*nIndex))*4095.0)/2);
  }
}

   
void setup()
{
 createSineTable();
   /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC4);
 
  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC1,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC 
  | TC_CMR_TCCLKS_TIMER_CLOCK2);
  TC_SetRC(TC1, 1, 238); // sets <> 44.1 Khz interrupt rate
  //TC_SetRC(TC1, 1, 109); // sets <>   96 Khz interrupt rate
 
  TC_Start(TC1, 1);
 
  // enable timer interrupts on the timer
  TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;
 
  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number 
  (=(1*3)+1) for timer1 channel1 */
  NVIC_EnableIRQ(TC4_IRQn);

   //CONFIGURACIÃƒâ€œN PARA CONEXIoN BLUETOOTH
    ModBluetooth.begin(9600);  
    Serial.begin(9600);  
    ModBluetooth.println("MODULO CONECTADO");  
    ModBluetooth.print("#");  

  //CONFIGURACION PARA LA SHIELD
    //Configuracion ADC
    ADC->ADC_MR |= 0x80;   // DAC en modo libre
    ADC->ADC_CR=2;         // Iniciar conversiÃƒÂ³n de ADCs
    ADC->ADC_CHER=0x1CC0;  // Activar ADCs mediante regristro
 
    //Configuracion DAC
    analogWrite(DAC0,0);  // Activar Dac 0
    analogWrite(DAC1,0);  // Activar Dac 1
 
    //Configuracion de usuario
    pinMode(LED, OUTPUT);  
    pinMode(FOOTSWITCH, INPUT_PULLUP);      
    pinMode(SWITCH, INPUT_PULLUP);  
    
    

  
}
 
void loop()
{


  
   //Leer ADCs
    while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// Esperar la conversiÃƒÂ³n de los ADC
    in_ADC0=ADC->ADC_CDR[7];               // leer datos ADC0
    in_ADC1=ADC->ADC_CDR[6];               // leer datos ADC1  
    if (fijar == 1)
    {
         POT0=ADC->ADC_CDR[10];                 // read data from ADC8        
         POT1=ADC->ADC_CDR[11];                 // read data from ADC9   
         POT2=ADC->ADC_CDR[12];                 // read data from ADC10    
    }

   //VERIFICAR CONEXION BLUETOOTH PARA PODER USAR LOS EFECTOS
    if (ModBluetooth.available())  
    { 
    char VarChar; //DATO QUE PERMITE SELECCIONAR EL EFECTO
    VarChar = ModBluetooth.read(); //LECTURA DE DATO
   
         int valor = ModBluetooth.read()-48;
          valor = map(valor, 0, 9, 0, 4095);
 
           potenciometro1 = valor;
       
        if(VarChar == 'a') 
        { 
         Dato=0;
         fijar=0;
         //ENVIAR MENSAJE A LA APLICACIÃƒâ€œN
          ModBluetooth.print("Seleccionado! CLEAN#"); 
          Serial.print("EFECTO CLEAN#"); 
           VarChar = ' ';

        } 
        if(VarChar == 'b') 
        { 
         Dato=1;
         fijar=0;
            //ENVIAR MENSAJE A LA APLICACIÃƒâ€œN
            ModBluetooth.print("Seleccionado! DISTORTION#"); 
            Serial.print("EFECTO DISTORTION#"); 
             VarChar = ' ';
        } 
        if(VarChar == 'c') 
        { 
         Dato=2;
         fijar=0;
         //ENVIAR MENSAJE A LA APLICACIÃƒâ€œN
           ModBluetooth.print("Seleccionado! ECHO#"); 
           Serial.print("EFECTO ECHON#"); 
            VarChar = ' ';
        } 
        if(VarChar == 'd') 
        { 
         Dato=3;
         fijar=0;
          //ENVIAR MENSAJE A LA APLICACIÃƒâ€œN
             ModBluetooth.print("Seleccionado! CHORUS#"); 
             Serial.print("EFECTO CHORUS#"); 
              VarChar = ' ';
        } 
        if(VarChar == 'e') 
        { 
         Dato=4;
         fijar=0;
         //ENVIAR MENSAJE A LA APLICACIÃƒâ€œN
          ModBluetooth.print("Seleccionado! TREMOLO#"); 
          Serial.print("EFECTO TREMOLO#");
           VarChar = ' ';
        } 
        if(VarChar == 'f') 
        { 
         Dato=5;
         fijar=0;
          //ENVIAR MENSAJE A LA APLICACIÃƒâ€œN
          ModBluetooth.print("Seleccionado! OCTAVADOR#"); 
          Serial.print("EFECTO OCTAVADOR#");
           VarChar = ' ';
        } 
          while (VarChar == 'x') 
        { 
         
          while(ModBluetooth.available()>0)
          {
            int valor = ModBluetooth.read()-48;
            valor = map (valor, 0, 9, 0, 4095);

            POT2 = valor;
            Serial.print(POT2);
            VarChar = ' ';
          }
        } 

        while (VarChar == 'y') 
        { 
         
          while(ModBluetooth.available()>0)
          {
            int valor = ModBluetooth.read()-48;
            valor = map (valor, 0, 9, 0, 4095);

            POT1 = valor;
            Serial.print(POT1);
            VarChar = ' ';
          }
        } 

        while (VarChar == 'z') 
        { 
         
          while(ModBluetooth.available()>0)
          {
            int valor = ModBluetooth.read()-48;
            valor = map (valor, 0, 9, 0, 4095);

            POT0 = valor;
            Serial.print(POT0);
            VarChar = ' ';
          }
        } 
        if (VarChar == 'p') 
        { 
         
         fijar = 1;
    VarChar = ' ';
        } 

}
}

 
void TC4_Handler()
{
  TC_GetStatus(TC1, 1);
    
    
   
  if (Dato == 0) // EFFECT 0: Volume-Booster    
  {

  //Encender el LED al activar el efecto!
  if (digitalRead(FOOTSWITCH)) digitalWrite(LED, HIGH); 
  else  digitalWrite(LED, LOW);
  

  //Ajustar Volumen con el Potenciometro 2
  out_DAC0=map(in_ADC0,0,4095,1,POT2);
  out_DAC1=map(in_ADC1,0,4095,1,POT2);    

  //Escribir en los dos DAC
  dacc_set_channel_selection(DACC_INTERFACE, 0);       //seleccionar DAC 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);// escribir en DAC 0
  dacc_set_channel_selection(DACC_INTERFACE, 1);       //seleccionar DAC 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);// escribir en DAC 1
  }
 
 
  else if (Dato==1)  // EFFECT 1: Asymmetric Distortion  
  {

  //Encender el LED al activar el efecto!
  if (digitalRead(FOOTSWITCH)) digitalWrite(LED, HIGH); 
  else  digitalWrite(LED, LOW);
  
  lim_sup=map(POT0,0,4095,4095,2047);
  lim_inf=map(POT1,0,4095,0000,2047);
 
  if(in_ADC0>lim_sup)  in_ADC0=lim_sup*1.5;
  if(in_ADC0<lim_inf)  in_ADC0=lim_inf*1.5;
 
  if(in_ADC1<lim_sup) in_ADC1=lim_sup*1.5;
  if(in_ADC1>lim_inf)  in_ADC1=lim_inf*1.5;
 
  //Ajustar Volumen con el Potenciometro 2
  out_DAC0=map(in_ADC0,0,4095,1,POT2);
  out_DAC1=map(in_ADC1,0,4095,1,POT2);
 
 
  //Escribir en los dos DAC
  dacc_set_channel_selection(DACC_INTERFACE, 0);       //seleccionar DAC 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);// escribir en DAC 0
  dacc_set_channel_selection(DACC_INTERFACE, 1);       //seleccionar DAC 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);// escribir en DAC 1
  }
    else if (Dato==2) // EFFECT 2: Echo.
  {
    //Encender el LED al activar el efecto!
  if (digitalRead(FOOTSWITCH)) digitalWrite(LED, HIGH); 
  else  digitalWrite(LED, LOW); 
     //Almacenar lecturas actuales
  muestraDelay[Contador_Delay]  = (in_ADC0 + (muestraDelay[Contador_Delay]))>>1;
 //                  y(n) = x(n) + a.x(n-D)  
  //Ajustar profundidad de delay basado en el potenciometro 0
  Profundidad_Delay =map(POT0>>2,0,2047,1,MAX_DELAY_A);
 
  //Aumentar/reiniciar contador de Delay
  Contador_Delay++;
  if(Contador_Delay >= Profundidad_Delay) Contador_Delay = 0; 
  out_DAC0 = ((muestraDelay[Contador_Delay]));
 
  //Agregar Volumen
  out_DAC0=map(out_DAC0,0,4095,1,POT2);
 
  //Escribir en los DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);          //Seleccionar DAC0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//escribir dac 0
   
  }
 
  else if (Dato==3)// EFFECT 3: Chourus
        
  {
 
 
  //Store current readings  
  sDelayBuffer0[DelayCounter] = in_ADC0;
 
  //Adjust Delay Depth based in pot0 position.
  POT0=map(POT0,0,1024,1,25); //25 empirically chosen
 
  DelayCounter++;
  if(DelayCounter >= Delay_Depth) 
  {
    DelayCounter = 0; 
    if(count_up)
    {
       if (digitalRead(FOOTSWITCH)) digitalWrite(LED, HIGH);
       for(p=0;p<POT0+1;p++) 
       sDelayBuffer0[Delay_Depth+p]=sDelayBuffer0[Delay_Depth-1]; 
       Delay_Depth=Delay_Depth+POT0;
       if (Delay_Depth>=MAX_DELAY_B)count_up=0;
    }
    else
    {
       digitalWrite(LED, LOW);
       Delay_Depth=Delay_Depth-POT0;
       if (Delay_Depth<=MIN_DELAY)count_up=1;
    }
  }

  
 
  out_DAC0 = sDelayBuffer0[DelayCounter];
 
  //Add volume control based in POT2
  out_DAC0=map(out_DAC0,0,4095,1,POT2);
 
  //Write the DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);       //select DAC channel 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC




  }

 else if (Dato==4) //EFECTO TREMOLO
 {
           //Aumentar frecuencia de tremolo
 POT0 = POT0>>1; //dividir valor entre 2 
 count++; 
 if (count>=160) //160 por ser el valor maximo del contador y se reinicia la cuenta d enuevo a 0
 {
   count=0;
   sample=sample+POT0;
   if(sample>=no_samples-POT0) 
   {
     sample=0;
     if(digitalRead(FOOTSWITCH)) //led efecto on, enciende cada que se reinicia el contador
     {
       digitalWrite(LED, !digitalRead(LED));
     }
     else
     {
       digitalWrite(LED, LOW);
     }
   }
 }
 
  //Crear señal de oscilador de baja frecuencia con control de profundidad en POT1
  LFO=map(nSineTable[sample],0,4095,(4095-POT1),4095);
 
  //Modular las señales de salida de la tabla de muestreo
  out_DAC0 =map(in_ADC0,1,4095,1, LFO);
  out_DAC1 =map(in_ADC1,1,4095,1, LFO);
 
  //Agregar volumen
  out_DAC0 =map(out_DAC0,1,4095,1, POT2);
  out_DAC1 =map(out_DAC1,1,4095,1, POT2);
 
   //escribir Dacs
  dacc_set_channel_selection(DACC_INTERFACE, 0);       
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);
  dacc_set_channel_selection(DACC_INTERFACE, 1);       
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);
          
 }

 else if (Dato==5) //EFECTO OCTAVER
 {
    //Encender el LED al activar el efecto!
  if (digitalRead(FOOTSWITCH)) digitalWrite(LED, HIGH); 
  else  digitalWrite(LED, LOW); 
//Almacenar lecturas actuales
  muestraDelay0[write_pt] = in_ADC0;
  muestraDelay1[write_pt] = in_ADC1;
 
  //Ajustar delay dependiendo en potenciometro 0
  Delay_Depth = MAX_DELAY-1;
 
  //Aumentar y reiniciar el contador
  write_pt++;
  if(write_pt >= Delay_Depth) write_pt = 0; 
 
  out_DAC0 = ((muestraDelay0[read_pt_A]));
  out_DAC1 = ((muestraDelay1[read_pt_B]));
 
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
 }






