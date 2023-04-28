
 #define ModBluetooth Serial1 //Variable para establecer los pines para la comunicacion seria. (serial1= 18 y 19)

int in_ADC0, in_ADC1;  //variables para ADC 0 y 1
int POT0, POT1, POT2, out_DAC0, out_DAC1; //Variables para los 3 potenciometros, variables para DACS
const int LED = 3;  //variable LED
const int FOOTSWITCH = 7; //variable FOOTSWITCH
const int SWITCH = 2; //variable Swtich 
int lim_sup, lim_inf;
 
void setup()
{
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
    POT0=ADC->ADC_CDR[10];                 // read data from ADC8        
  POT1=ADC->ADC_CDR[11];                 // read data from ADC9   
  POT2=ADC->ADC_CDR[12];                 // read data from ADC10 


 
}

void tucola(){

 while(ModBluetooth.available()>0){
    int valor = ModBluetooth.read()-48;
  valor = map(valor, 0, 9, 0, 4095);
  Serial.println(valor);
 
  
   //Ajustar Volumen con el Potenciometro 2
  out_DAC0=map(in_ADC0, 0, 4095, 1,valor);
  out_DAC1=map(in_ADC1,0,4095,1,valor);

  delay (200);
    //Escribir en los dos DAC
  dacc_set_channel_selection(DACC_INTERFACE, 0);       //seleccionar DAC 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);// escribir en DAC 0
 
      dacc_set_channel_selection(DACC_INTERFACE, 1);       //seleccionar DAC 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);// escribir en DAC 1
    }
 
  

}

