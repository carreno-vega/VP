#include <Wire.h> 
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <SoftwareSerial.h>


LiquidCrystal_I2C lcd(0x27,20,4); 
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);  // Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x61); // Create the motor shield object with the 0x61 I2C address

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

Adafruit_DCMotor *myMotor2_1 = AFMS1.getMotor(1);
Adafruit_DCMotor *myMotor2_2 = AFMS1.getMotor(2);
Adafruit_DCMotor *myMotor2_3 = AFMS1.getMotor(3);
Adafruit_DCMotor *myMotor2_4 = AFMS1.getMotor(4);

/*Varibles de relé*/
int  actuador_D8  = 8;   //Relé 1 D8  en arduino_xbee
int  actuador_D9  = 9;   //Relé 2 D9  en arduino_xbee
int  actuador_D10 = 10;  //Relé 3 D10 en arduino_xbee
int  actuador_D11 = 11;  //Relé 4 D11 en arduino_xbee

int estado_manual_1 = 0;
int estado_manual_2 = 0;
int estado_manual_3 = 0;
int estado_manual_4 = 0;

int flag_control_automatico;

int sens0_pros;
int sens1_pros;
int sens2_pros;
int sens3_pros;
int sens4_pros;
int sens5_pros;

int sp_ph_b1_on;
int sp_ph_b1_off;
int sp_ph_b2_on;
int sp_ph_b2_off;
int sp_od_ag_on;
int sp_od_ag_off;

byte sp_ph_b1_on_h;    
byte sp_ph_b1_on_l;
byte sp_ph_b1_off_h;
byte sp_ph_b1_off_l;    
byte sp_ph_b2_on_h;
byte sp_ph_b2_on_l;
byte sp_ph_b2_off_h;
byte sp_ph_b2_off_l;
byte sp_od_ag_on_h;
byte sp_od_ag_on_l;
byte sp_od_ag_off_h;
byte sp_od_ag_off_l;

float sens0_actual;
float sens1_actual;
float sens2_actual;
float sens3_actual;
float sens4_actual;
float sens5_actual;

float sens0_anterior;
float sens1_anterior;
float sens2_anterior;
float sens3_anterior;
float sens4_anterior;
float sens5_anterior;

float sensor_return;

byte  sof_tx;
byte  id_trama_tx;  // normal = 1, confirm = 2
byte  estado_tx; 

byte sens0_tx_h;
byte sens0_tx_l;
byte sens1_tx_h;
byte sens1_tx_l;
byte sens2_tx_h;
byte sens2_tx_l;
byte sens3_tx_h;
byte sens3_tx_l;
byte sens4_tx_h;
byte sens4_tx_l;
byte sens5_tx_h;
byte sens5_tx_l;
byte aux_tx;
byte aux_tx1;
byte aux_tx2;
byte eof_tx;

byte  id_trama;
byte  estado_rx;

float  arreglo[19];

/* Variables Control Manual*/
byte  inst1_man;
byte  inst2_man;  
byte  inst3_man;  
byte  inst4_man; 
byte  inst5_man;  
byte  inst6_man;
byte  inst7_man;  
byte  inst8_man;  
byte  inst9_man;
byte  inst10_man; 
byte  inst11_man; 
byte  inst12_man;
byte  aux1_man;
byte  aux2_man;  
byte  aux3_man;

/* Variables de calibracion*/
byte flag_cal;  //Indica el sensor a calibrar
byte val_cal1;  
byte val_cal2;  
byte val_cal3;  
byte val_cal4;  
byte val_cal5;
byte val_cal6;  
byte val_cal7;  
byte val_cal8; 
byte val_cal9;  
byte val_cal_10;  
byte val_cal_11; 
byte aux1_cal;  //Para ph, aux1_cal=1 -> pH=4, aux1_cal=2 ->pH=7
byte aux2_cal;  
byte aux3_cal; 

int contador_eeprom;
int eeprom_tasa;
int sector_init;

int  contador;

byte flag_inicio;

/*Variable estado salidas digitales*/
byte estado_led;
byte estado_led1;
byte estado_led2;
byte estado_led3;
byte output_state;

/*Variables propias del sensor pH*/
float sensor_ph_value;
float sensor_ph_value_float;
float sensor_ph_value_volt;
byte flag_ph1;
byte flag_ph2;
byte flag_ph3;
float sensor_ph_4_cal;
float sensor_ph_7_cal;
float sensor_ph_10_cal;
float paso_ph_cal;

float actual_ph,anterior_ph;

/*variables controlador pH*/
float controlador_ph_value;
byte flag_ph1_cont;
byte flag_ph2_cont;
byte flag_ph3_cont;
float controlador_ph4_cal;
float controlador_ph7_cal;
float controlador_ph10_cal;
float paso_ph_cont;
/*Variables propias del sensor de temperatura PTC*/
float sensor_temp_value;
float sensor_temp_value_volt;
float sensor_temp_value_float;
float sensor_ptc_value;

/*Variables timer*/
byte timer_lectura;
int  timer_control;
byte timer_muestreo;
byte milisegundos;
byte segundos;
byte minutos;

/**/
float sens0_read;
float sens1_read;
float sens2_read;
float sens3_read;
float sens4_read;
float sens5_read;

/*Declaracion entradas analogas para sensores*/
byte ADC0 = A0;   
byte ADC1 = A1;   
byte ADC2 = A2;   
byte ADC3 = A3;   
byte ADC4 = A4;   
byte ADC5 = A5;   

/*Timer1*/
int  aux_timer1; 
int  seconds;
int  timer_loop;

float voltaje_ref_ADC;

byte ok_calibration;
int aux_eeprom;

/*id_trama enviada incrementa cada 1 seg*/
byte incrementador_tx; // incrementador de timer enviado hacia java

byte data_motor[6];

int data_cero;

void setup()
{  
  /**************************************************/
  //  SET TIMER  //
  //Since Timer1 is 16 bits, it can hold a maximum value of (2^16 – 1) or 65535 a 16MHz.
  cli();             // Desaciva las interrupciones globales
  TCCR1A  = 0;       // pone el regitro TCCR1A entero a 0
  TCCR1B  = 0;       // pone el registro TCCR1B entero a 0
  OCR1A   = 624;     // configurado para 0.008 seg (125 Hz) OCR1A = 124, Comparación cada un seg 15624,   www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
  /*
  we divide our clock source by 1024. This gives us a timer resolution of 1/(16*10^6 / preescaler), 
  (# timer counts + 1) = (target time) / (timer resolution)
  (# timer counts + 1) = (0.01) / (1.6e-5 s)
  (# timer counts + 1) = 625
  (# timer counts) = 625 - 1 = 624  El timer1 iguala este valor para activar la interrupecion, para 0.01 seg es 624
  */
  TCCR1B |= (1 << WGM12);
  //TCCR1B |= (1 << CS10);  //CS12 = 1 CS11 = 0 CS10 = 1, PREESCALER = 1024
  TCCR1B |= (1 << CS12);   //CS12 = 1 CS11 = 0 CS10 = 0, PREESCALER = 256
  TIMSK1 |= (1 << OCIE1A);  //datasheet 32U4 
  sei();
  //  FIN SET TIMER  //
  /***************************************************/
  
  Serial.begin(9600);
  Serial1.begin(9600);
  
  AFMS.begin(1000);  // frecuencia en hertz, máximo 1600 Hz
  AFMS1.begin(1000);  // frecuencia en hertz, máximo 1600 Hz
  
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);
  
  myMotor2_1->run(FORWARD);
  myMotor2_2->run(FORWARD);
  myMotor2_3->run(FORWARD);
  myMotor2_4->run(FORWARD);
  
  pinMode(actuador_D8, OUTPUT);    // Agitador
  pinMode(actuador_D9, OUTPUT);    // Actuador 1
  pinMode(actuador_D10, OUTPUT);   // Actuador 2
  pinMode(actuador_D11, OUTPUT);   // Actuador 3
  
  // PWM0  : Bomba 1
  // PWM1  : Bomba 2
  // PWM14 : Bomba 3
  // PWM15 : Bomba 4
  
  digitalWrite(actuador_D8,LOW);  
  digitalWrite(actuador_D9,LOW);
  digitalWrite(actuador_D10,LOW);
  digitalWrite(actuador_D11,LOW);
  
  flag_control_automatico = 0;
  data_cero = 0;
}
void Motor_conectar()
{
//  Serial.write("Conectar");
  delay(100); 
  Serial1.write(254);  
  delay(100); 
  Serial1.write(160);
  delay(100);
  Serial1.write(data);
  delay(100);
  Serial1.write(data);
  delay(100);
  Serial1.write(data);
  delay(100);
  Serial1.write(160);
}
void Motor_set_RPM(int high, int low)
{
  int checksum = (177 + high + low) & 0xff;
  
  Serial1.write(254);  
  delay(100); 
  Serial1.write(177);
  delay(100);
  Serial1.write(high);
  delay(100);
  Serial1.write(low);
  delay(100);
  Serial1.write(data_cero);
  delay(100);
  Serial1.write(checksum);
}

void set_config_actuadores(byte s_id, byte s_rpm) //actuador_conf, valor_conf_h, valor_conf_l
{
    /*
    Actuador_conf:
    
    1: Agitador
    2: Actuador 1
    3: Actuador 2
    4: Actuador 3
    5: Bomba 1
    6: Bomba 2
    7: Bomba 3
    8: Bomba 4
    */
    
  switch(s_id)
  {
    case(1):    // Agitador
    {
      // Serial
      int rpm_h = (s_pm >> 8) & 0xff;
      int rpm_l = s_rpm & 0xff;
      Motor_Set_RPM(rpm_h,rpm_l);  // RPM High, RPM Low
      delay(500);
      Motor_Set_RPM(rpm_h,rpm_l);  // RPM High, RPM Low 
      break;
    }
    case(2):    // Actuador 1
    {
      break;
    }
    case(3):    // Actuador 2
    {
      break;
    }
    case(4):    // Actuador 3
    {
      break;
    }
    case(5):    // Bomba 1
    {
      // Motor 1 -> Bomba 1
      // Bomba 1: RPM Max 
      int rpm_max = 1000;
      int rpm_dec = ((255 * (s_rpm - rpm_max)) / rpm_max) + 255; 
      myMotor1->setSpeed(rpm_dec);  // 0 - 255
      break;
    }
    case(6):    // Bomba 2
    {
      // Motor 2 -> Bomba 2
      int rpm_max = 1000;
      int rpm_dec = ((255 * (s_rpm - rpm_max)) / rpm_max) + 255;
      myMotor2->setSpeed(rpm_dec);  // 0 - 255
      break;
    }
    case(7):    // Bomba 3
    {
      // Motor 3 -> Bomba 3
      int rpm_max = 1000;
      int rpm_dec = ((255 * (s_rpm - rpm_max)) / rpm_max) + 255;
      myMotor3->setSpeed(rpm_dec);  // 0 - 255
      break;
    }
    case(8):    // Bomba 4
    {
      // Motor 4 -> Bomba 4
      int rpm_max = 1000;
      int rpm_dec = ((255 * (s_rpm - rpm_max)) / rpm_max) + 255;
      myMotor4->setSpeed(rpm_dec);  // 0 - 255
      break;
    }
    default:break;
  }  
}

void control_automatico(void)  // Válido sólo para pH
{
  // sens0 -> ADC0: PTC - NTC 
  // sens1 -> ADC1: pH
  // sens2 -> ADC2: 4-20 mA (3)
  // sens3 -> ADC3: OD
  // sens4 -> ADC4: 4-20 mA (1)
  // sens5 -> ADC5: 4-20 mA (2)
  
  // actuador_D8  -> Agitador
  // actuador_D9  -> Actuador 1
  // actuador_D10 -> Actuador 2
  // actuador_D11 -> Actuador 3
  
  // PWM0  : Bomba 1
  // PWM1  : Bomba 2
  // PWM14 : Bomba 3
  // PWM15 : Bomba 4
  
  if(sens1_pros >= sp_ph_b1_on)
  {
  //digitalWrite(11,HIGH);  // sens0_act_asi por D9
    AFMS.setPWM(0, 4096);     // Salida alta Bomba 1
  }  
  if(sens1_pros <= sp_ph_b1_off)
  {
  //digitalWrite(11,LOW);  // sens0_act_asi por D9
    AFMS.setPWM(0, 4096);    // Salida baja Bomba 1
  } 
  if(sens1_pros <= sp_ph_b2_on)
  {
  //digitalWrite(10,HIGH);   // sens0_act_asi  
    AFMS.setPWM(1, 4096);    // Salida alta Bomba 2
  }
  if(sens1_pros >= sp_ph_b2_off)
  {
  //digitalWrite(10,LOW);    // sens0_act_asi
    AFMS.setPWM(1, 4096);    // Salida baja Bomba 2
  }
  if(sens3_pros <= sp_od_ag_on)
  {
    digitalWrite(8,HIGH);    // Salida alta Agitador
  }
  if(sens3_pros >= sp_od_ag_off)
  {
    digitalWrite(8,LOW);     // Salida baja Agitador
  }  
}

void control_manual(int flag_control_m)
{
  // Controla manualmente el funcionamiento de los actuadores
  // Hay 2 tipos de control: 
  // 1 -> Control de los relés de la tarjeta
  // 2 -> Control mediante salidas digitales 
  switch(flag_control_m)
  {
    /********************************************/
    // Actuador TIPO 1 //
    case(1):  // Agitador OFF
    { 
      digitalWrite(actuador_D8,LOW);
      break;
    }
    case(2):  // Agitador ON
    { 
      digitalWrite(actuador_D8,HIGH);
      break;
    }
    case(3):  // Actuador 1 OFF
    { 
      digitalWrite(actuador_D9,LOW);
      break;
    }
    case(4):  // Actuador 1 ON
    { 
      digitalWrite(actuador_D9,HIGH);
      break;
    }
    case(5):  // Actuador 2 OFF
    { 
      digitalWrite(actuador_D10,LOW);
      break;
    }
    case(6):  // Actuador 2 ON
    { 
      digitalWrite(actuador_D10,HIGH);
      break;
    }
    case(7):  // Actuador 3 OFF
    { 
      digitalWrite(actuador_D11,LOW);
      break;
    }
    case(8):  // Actuador 3 ON
    { 
      digitalWrite(actuador_D11,HIGH);
      break;
    }
    /********************************************/
    // Actuador TIPO 2 //
    case(9):  // Bomba 1 OFF
    { 
      AFMS.setPWM(0, 0);
      estado_manual_1 = 0;
      break;
    }
    case(10):  // Bomba 1 ON
    { 
      AFMS.setPWM(0, 4096);
      estado_manual_1 = 1;
      break;
    }
    case(11):  // Bomba 2 OFF
    { 
      AFMS.setPWM(1, 0);
      estado_manual_2 = 0;
      break;
    }
    case(12):  // Bomba 2 ON
    { 
      AFMS.setPWM(1, 4096);
      estado_manual_2 = 1;
      break;
    }
    case(13):  // Bomba 3 OFF
    { 
      AFMS.setPWM(14, 0);
      estado_manual_3 = 0;
      break;
    }
    case(14):  // Bomba 3 ON
    { 
      AFMS.setPWM(14, 4096);
      estado_manual_3 = 1;
      break;
    }
    case(15):  // Bomba 4 OFF
    { 
      AFMS.setPWM(15, 0);
      estado_manual_4 = 0;
      break;
    }
    case(16):  // Bomba 4 ON
    { 
      AFMS.setPWM(15, 4096);
      estado_manual_4 = 1;
      break;
    }
    /********************************************/
    default:break;
  }   
}

    // PROCESAR DATOS //
    /********************************************/
  // ADC0: PTC - NTC 
  // ADC1: pH
  // ADC2: 4-20 mA (3)
  // ADC3: OD
  // ADC4: 4-20 mA (1)
  // ADC5: 4-20 mA (2)
  
float procesar_PT(float sensor_in)
{
  sens0_actual = 0.01 * sensor_in + sens0_anterior * 0.99;
  sens0_anterior = sens0_actual;  
  return sens0_actual;
}
float procesar_PH(int flag, float sensor_in)
{
  sens1_actual = 0.05 * sensor_in + sens1_anterior * 0.95;
  sens1_anterior = sens1_actual; 
  return sens1_actual; 
}
float procesar_420_3(int flag, float sensor_in)
{
  sens2_actual = 0.05 * sensor_in + sens2_anterior * 0.95;
  sens2_anterior = sens2_actual; 
  return sens2_actual; 
}
float procesar_OD(int flag, float sensor_in)
{
  sens3_actual = 0.01 * sensor_in + sens3_anterior * 0.99;
  sens3_anterior = sens3_actual;
  return sens3_actual; 
}
float procesar_420_1(int flag, float sensor_in)
{
  sens4_actual = 0.01 * sensor_in + sens4_anterior * 0.99;
  sens4_anterior = sens4_actual;
  return sens4_actual;  
}
float procesar_420_2(int flag, float sensor_in)
{
  sens5_actual = 0.01 * sensor_in + sens5_anterior * 0.99;
  sens5_anterior = sens5_actual;
  return sens5_actual;
}


void actuadores_off(void)
{
  digitalWrite(actuador_D8,LOW); 
  digitalWrite(actuador_D9,LOW);
  digitalWrite(actuador_D10,LOW);
  digitalWrite(actuador_D11,LOW);

}
void deco_trama(void)
{
/* ID TRAMA
1	Trama de Control Automático para actuador
2	Trama de Control Manual
3	Trama de Calibraciòn
4       Trama de inicio de programa
5       Trama de apagado de actuadores
6       Trama de error recepción de datos
7       Trama de lectura valores de calibración EEPROM
8       Trama de configuración
9       Trama de reinicio valor de variables
*/
  id_trama    = arreglo[1];
  estado_rx   = arreglo[2];   // Incrementador
  switch(id_trama)
  {
    case(1): //trama de control automático
    {  
      sp_ph_b1_on_h   = arreglo[3];    //ADC el numero de la entrada (1 a 6) donde esta conectado el actuador
      sp_ph_b1_on_l   = arreglo[4];
      sp_ph_b1_off_h  = arreglo[5];
      sp_ph_b1_off_l  = arreglo[6];    
      sp_ph_b2_on_h   = arreglo[7];
      sp_ph_b2_on_l   = arreglo[8];
      sp_ph_b2_off_h  = arreglo[9];
      sp_ph_b2_off_l  = arreglo[10];
      sp_od_ag_on_h   = arreglo[11];
      sp_od_ag_on_l   = arreglo[12];
      sp_od_ag_off_h  = arreglo[13];
      sp_od_ag_off_l  = arreglo[14];
      
      // Todos los valores de pH son trabajados en decimal
      // El valor de pH está inveramente relacionado al valor decimal.
      
      sp_ph_b1_on  = (sp_ph_b1_on_h  << 8) + sp_ph_b1_on_l;    // Bajo este valor se prende bomba base
      sp_ph_b1_off = (sp_ph_b1_off_h << 8) + sp_ph_b1_off_l ;  // Sobre este valor se apaga bomba base
      sp_ph_b2_on  = (sp_ph_b2_on_h << 8 ) + sp_ph_b2_on_l;    // Sobre este valor se prende bomba ácido
      sp_ph_b2_off = (sp_ph_b2_off_h << 8) + sp_ph_b2_off_h;   // Bajo este valor se apaga bomba ácido
      sp_od_ag_on  = (sp_od_ag_on_h << 8) + sp_od_ag_on_l;     // Bajo este valor se prende agitador
      sp_od_ag_off = (sp_od_ag_off_h << 8) + sp_od_ag_off_l;   // Sobre este valor se apaga agitador

      break;                     
    }
    case(2): //trama de control manual
    { 
    /*
    
      Posibles valores para -> inst1_man:
      
      1: Agitador OFF
      2: Agitador ON
      3: Actuador 1 OFF
      4: Actuador 1 ON
      5: Actuador 2 OFF
      6: Actuador 2 ON
      7: Actuador 3 OFF
      8: Actuador 3 ON
      9: Bomba 1 OFF
      10: Bomba 1 ON
      11: Bomba 2 OFF
      12: Bomba 2 ON
      13: Bomba 3 OFF
      14: Bomba 3 ON
      15: Bomba 4 OFF
      16: Bomba 4 ON
      
    */
      
      inst1_man     = arreglo[3];   // Variable de información para control
      inst2_man     = arreglo[4];  
      inst3_man     = arreglo[5];  
      inst4_man     = arreglo[6];   
      inst5_man     = arreglo[7];   
      inst6_man     = arreglo[8];   
      inst7_man     = arreglo[9];   
      inst8_man     = arreglo[10];  
      inst9_man     = arreglo[11];  
      inst10_man    = arreglo[12];  
      inst11_man    = arreglo[13];  
      inst12_man    = arreglo[14];  
      aux1_man      = arreglo[15];  
      aux2_man      = arreglo[16];  
      aux3_man      = arreglo[17];
      break;
    }
    case(3):  // trama = calibracion
    { 
      flag_cal   = arreglo[3];  // 1: pH, 2: OD, 3: Temp. Indica el sensor a calibrar
      val_cal1   = arreglo[4];  
      val_cal2   = arreglo[5];  
      val_cal3   = arreglo[6];  
      val_cal4   = arreglo[7];  
      val_cal5   = arreglo[8];  
      val_cal6   = arreglo[9];  
      val_cal7   = arreglo[10];  
      val_cal8   = arreglo[11]; 
      val_cal9   = arreglo[12];  
      val_cal_10 = arreglo[13];  
      val_cal_11 = arreglo[14]; 
      aux1_cal   = arreglo[15];  //1: pH = 4, 2: pH = 7, 3: OD = 0, 4: OD = 100, 5: Temp = 1, 6: Temp = 2 
      aux2_cal   = arreglo[16];  
      aux3_cal   = arreglo[17]; 
      break;
    }
    case(4):  // Trama de configuración de actuadores (RPM Bombas)
    {   
      /*
      Actuador_conf:
      
      1: Agitador
      2: Actuador 1
      3: Actuador 2
      4: Actuador 3
      5: Bomba 1
      6: Bomba 2
      7: Bomba 3
      8: Bomba 4
      
      Valor de configuración:
      
      Valor_conf_h
      Valor_conf_l           
      */
      
      actuador_conf  = arreglo[3];   // Variable de información de actuador
      valor_conf_h   = arreglo[4];   // Variable de valor de configuración High
      valor_conf_l   = arreglo[5];   // Variable de valor de configuración Low
      inst4_conf     = arreglo[6];   
      inst5_conf     = arreglo[7];   
      inst6_conf     = arreglo[8];   
      inst7_conf     = arreglo[9];   
      inst8_conf     = arreglo[10];  
      inst9_conf     = arreglo[11];  
      inst10_conf    = arreglo[12];  
      inst11_conf    = arreglo[13];  
      inst12_conf    = arreglo[14];  
      aux1_conf      = arreglo[15];  
      aux2_conf      = arreglo[16];  
      aux3_conf      = arreglo[17];
      break;
    
    }
    case(7):                      // Trama para leer datos de calibración desde EEPROM. Sólo se procesa con ID_TRAMA
    {
      break;
    }
    case(9):                      // Trama de reinicio. Sólo se procesa con ID_TRAMA
    {
      break;
    }
    case(10):                     // Trama de Data logger. Sólo se procesa con ID_TRAMA
    {
      break;
    }
    case(14):
    {
      eeprom_tasa  = arreglo[3];  // Tiempo en minutos almacenamiento en datalogger
      break;
    }
    case(15):                      // Trama de check_com. Sólo se procesa con ID_TRAMA
    {
      break;
    }
    default:break;  
  }
}
int read_trama()
{
  arreglo[contador] = Serial.read();   
  if(arreglo[contador] == '#')                                    
  {
    do                                                 
    {
      contador++;
      arreglo[contador] = Serial.read();                    
    }
    while(contador < 18);  // largo trama 19  
    if((arreglo[0] == '#') && (arreglo[18] == '%'))
    {
      contador = 0;  
      return(1);
    }
    else
    {
      contador = 0;
      return(0);      
    }
  } 
}
void variables_reset()
{

}
byte set_calibracion(byte flag_c)
{
  switch(flag_c)
  {
    case(1):  // Calibracion sensor de pH
    { 
      if(aux1_cal == 1) //calibracion pH4
      {
        sensor_ph_4_cal = sensor_ph_value_volt; //lectura pH4 de bit a milivolts     
        int ph4_dec = actual_ph; //0 a 1023
        byte ph4_dec_lsb = 0xFF & ph4_dec;
        byte ph4_dec_msb = 0xFF & (ph4_dec >> 8);
        EEPROM.write(12,ph4_dec_lsb);
        EEPROM.write(14,ph4_dec_msb);
        // END SET CAL VAL ON EEMPROM
        flag_ph1 = 1;
       // digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 2) //calibracion con pH7
      {
        sensor_ph_7_cal = sensor_ph_value_volt; //lectura pH4 de bit a milivolts
        // SET CAL VAL ON EEMPROM
        int ph7_dec = actual_ph; //0 a 1023
        byte ph7_dec_lsb = 0xFF & ph7_dec;
        byte ph7_dec_msb = 0xFF & (ph7_dec >> 8);
        EEPROM.write(16,ph7_dec_lsb);
        EEPROM.write(18,ph7_dec_msb);
        flag_ph2 = 1;
       // digitalWrite(led2,HIGH);
      }
      
      if((flag_ph1 == 1) && (flag_ph2 == 1))
      {
        paso_ph_cal = ((7 - 4) / (sensor_ph_7_cal - sensor_ph_4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1 = 0;
        flag_ph2 = 0;
      }
      
      break;
    }
    case(2):  // Calibracion Oxigeno Disuelto
    { 
      break;
    }
    case(3):  // Calibracion Controlador pH 420 (1)
    { 
      if(aux1_cal == 4) //calibracion pH4
      {
        controlador_ph4_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph1_cont = 1;
       // digitalWrite(led1,HIGH);
      }
      else if(aux1_cal == 5) //calibracion con pH7
      {
        controlador_ph7_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph2_cont = 1;
        //digitalWrite(led2,HIGH);
      }
      
      if((flag_ph1_cont == 1) && (flag_ph2_cont == 1))
      {
        paso_ph_cont = ((7 - 4) / (controlador_ph7_cal - controlador_ph4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1_cont = 0;
        flag_ph2_cont = 0;
        //digitalWrite(led3,HIGH);
      }
      break;     
    }
    case(4):  // Calibracion Controlador auxiliar 420 (2)
    { 
      break;
    }
    case(5):  // Calibracion sensor_ptc
    { 
      break;
    }
     case(6):  // Calibracion auxiliar
    { 
      break;
    }
    default:break;
  }
} 
void lectura_sensores()
{ 
  // ADC0: PTC - NTC 
  // ADC1: pH
  // ADC2: 4-20 mA (3)
  // ADC3: OD
  // ADC4: 4-20 mA (1)
  // ADC5: 4-20 mA (2)
    
     
  switch(timer_lectura)             // Lectura de un sensor cada 0.1 (s), equivale a un tiempo de lectura = 0.6 (s) x sensor,  fs= (1/0.6)
  {
    // ADC0: PTC - NTC 
    // ADC1: pH
    // ADC2: 4-20 mA (3)
    // ADC3: OD
    // ADC4: 4-20 mA (1)
    // ADC5: 4-20 mA (2)
    
    case(1):
    {
      sens0_read = analogRead(ADC0);             // PTC - NTC
    //sens0_pros = procesar_datos(0,sens0_read); // 60 ms   
      sens0_pros = procesar_PT(sens0_read); 
      break;
    }
    case(2):
    {
      sens1_read = analogRead(ADC1);             // pH
      sens1_pros = procesar_PH(sens1_read); // 60 ms 
    //  procesar_datos(2);
      break;
    }
    case(3):
    {
      sens2_read = analogRead(ADC2);             // 4-20 (3)
      sens2_pros = procesar_420_3(sens2_read); // 60 ms 
     // procesar_datos(3);
      break;
    }
    case(4):
    {
      sens3_read = analogRead(ADC3);             // OD
      sens3_pros = procesar_OD(sens3_read); // 60 ms 
     // procesar_datos(4);
      break;
    }
    case(5):
    {
      sens4_read = analogRead(ADC4);                // 4-20 (1)
      sens4_pros = procesar_420_1(sens4_read);
    //  procesar_datos(5);
      break;
    }
    case(6):
    {
      sens5_read = analogRead(ADC5);                // 4-20 (2)
      sens5_pros = procesar_420_2(sens5_read);
      timer_lectura = 0;
      break;
    }
    default:
    {
      timer_lectura = 0;
      break;
    }
  }
}
void estado_output(void)  // Estado de los Rele - salidas digitales
{
 // bitWrite(x, n, b)
 // x: the numeric variable to which to write
 // n: which bit of the number to write, starting at 0 for the least-significant (rightmost) bit
 // b: the value to write to the bit (0 or 1)
  
  output_state = 0;
  
  estado_led  = digitalRead(actuador_D8);
  bitWrite(output_state, 0, estado_led); 
  estado_led1 = digitalRead(actuador_D9);
  bitWrite(output_state, 1, estado_led1);
  estado_led2 = digitalRead(actuador_D10);
  bitWrite(output_state, 2, estado_led2);
  estado_led3 = digitalRead(actuador_D11);
  bitWrite(output_state, 3, estado_led3); 
  
  bitWrite(output_state, 4, estado_manual_1);  // Será 0 cuando se apague el PWM0. Seá 1 cuando se prenda PWM0
  bitWrite(output_state, 5, estado_manual_2);  // Idem con PWM1
  bitWrite(output_state, 6, estado_manual_3);  // Idem con PWM14
  bitWrite(output_state, 7, estado_manual_4);  // Idem con PWM15
}
byte make_trama(byte a,byte b)
{
/* ID TRAMA
1	Trama de Control Automático para actuador
2	Trama de Control Manual
3	Trama de Calibraciòn
4       Trama de inicio de programa
5       Trama de apagado de actuadores
6       Trama de error recepción de datos
7       Trama de lectura valores de calibración EEPROM
8       Trama de configuración
9       Trama de reinicio valor de variables
*/

  sof_tx      = '#';
  id_trama_tx = a;                          // 
  estado_tx   = b;                          // envia estado q se recibiò o incrementador para trama normal
        
  sens0_tx_h    = 0xFF & (sens0_pros >> 8);               //
  sens0_tx_l    = 0xFF & sens0_pros;
  sens1_tx_h    = 0xFF & (sens1_pros >> 8);               //
  sens1_tx_l    = 0xFF & sens1_pros;
  sens2_tx_h    = 0xFF & (sens2_pros >> 8);               //
  sens2_tx_l    = 0xFF & sens2_pros;
  sens3_tx_h    = 0xFF & (sens3_pros >> 8);               //
  sens3_tx_l    = 0xFF & sens3_pros;
  sens4_tx_h    = 0xFF & (sens4_pros >> 8);               //
  sens4_tx_l    = 0xFF & sens4_pros;
  sens5_tx_h    = 0xFF & (sens5_pros >> 8);               //
  sens5_tx_l    = 0xFF & sens5_pros;
  
  aux_tx        = output_state;               //Estado de los relay (Encendido = 1 / Apagado = 0)
  aux_tx1       = 0;
  aux_tx2       = 0;
  eof_tx        = '%';
}
void make_eeprom_trama(byte a,int b)  // a = id_trama = 11 (envio de datos eeprom) , b = i del bucle for
{ 
  sof_tx      = '#';
  id_trama_tx = a;                          // a = id_trama = 11 
  estado_tx   = b;                          // envia estado q se recibiò o incrementador para trama normal 
  sens0_tx_h    = 0xFF & EEPROM.read(10 + b * 12);               //
  sens0_tx_l    = 0xFF & EEPROM.read(11 + b * 12);
  sens1_tx_h    = 0xFF & EEPROM.read(12 + b * 12);               //
  sens1_tx_l    = 0xFF & EEPROM.read(13 + b * 12);
  sens2_tx_h    = 0xFF & EEPROM.read(14 + b * 12);               //
  sens2_tx_l    = 0xFF & EEPROM.read(15 + b * 12);
  sens3_tx_h    = 0xFF & EEPROM.read(16 + b * 12);              //
  sens3_tx_l    = 0xFF & EEPROM.read(17 + b * 12);
  sens4_tx_h    = 0xFF & EEPROM.read(18 + b * 12);               //
  sens4_tx_l    = 0xFF & EEPROM.read(19 + b * 12);
  sens5_tx_h    = 0xFF & EEPROM.read(20 + b * 12);              //
  sens5_tx_l    = 0xFF & EEPROM.read(21 + b * 12);
  
  aux_tx        = 0;               
  aux_tx1       = 0;
  aux_tx2       = 0;
  eof_tx        = '%'; 
}

void make_trama_actuadores(byte a, byte b, byte c)
{
    sof_tx      = '#';
  id_trama_tx   = a;                          // a = id_trama = 11 
  estado_tx     = b;                          // envia estado q se recibiò o incrementador para trama normal 
  sens0_tx_h    = c;              
  sens0_tx_l    = 0;
  sens1_tx_h    = 0;             
  sens1_tx_l    = 0;
  sens2_tx_h    = 0;              
  sens2_tx_l    = 0;
  sens3_tx_h    = 0;              
  sens3_tx_l    = 0;
  sens4_tx_h    = 0;           
  sens4_tx_l    = 0;
  sens5_tx_h    = 0;            
  sens5_tx_l    = 0;
  aux_tx        = 0;               
  aux_tx1       = 0;
  aux_tx2       = 0;
  eof_tx        = '%'; 
}
void send_trama()
{
  Serial.write(sof_tx);
  Serial.write(id_trama_tx);
  Serial.write(estado_tx);
  Serial.write(sens0_tx_h);
  Serial.write(sens0_tx_l);
  Serial.write(sens1_tx_h);
  Serial.write(sens1_tx_l);
  Serial.write(sens2_tx_h);
  Serial.write(sens2_tx_l);
  Serial.write(sens3_tx_h);
  Serial.write(sens3_tx_l);
  Serial.write(sens4_tx_h);
  Serial.write(sens4_tx_l);
  Serial.write(sens5_tx_h);
  Serial.write(sens5_tx_l);
  Serial.write(aux_tx);
  Serial.write(aux_tx1);
  Serial.write(aux_tx2);
  Serial.write(eof_tx); 
}
ISR(TIMER1_COMPA_vect)   //Flag correspondiente a timer1 comparacion
{                        
    aux_timer1 = 1;     
}

void loop()
{
  if(timer_loop == 10)             //cada 100 (ms)
  {
    if (Serial.available() > 0)    // Si es que hay datos en el buffer Serial.
    {   
      if(read_trama())             // Devuelve 1 si la estructura de la trama está ok
      {
        deco_trama();              // Decodifica la trama en función del ID_TRAMA y asigna valores a variables correspondientes.
        estado_output();           // Estado de la salidas digitales HIGH = 1 o LOW = 0  valores almacenados en variable output_state.
      }
      else
      {
        make_trama(6,0);           // Prepara trama de error ID = 6
        send_trama();              // Envía trama de error. 
      }       
    }
    
    if (Serial1.available() > 0)   // Lectura datos de motor
    {
      do
      {
        data_motor[aux] = Serial1.read();  // Lee en byte. Si se escribe por terminal "1" en ASCII es equivalente a 49 decimal.
        aux++;
      }while(Serial1.available() > 0);     // Recibe 6 bytes. 
      aux = 0;
    }
     
    switch(id_trama)               // Procesa en función del ID_TRAMA recibido
    {
      case(1):                     // Trama de control automático
      {
        make_trama(1,0);           // Prepara trama de respuesta. Prepara trama de respuesta (1 -> ID trama idem RX, 0 -> incrementador no importa)
        send_trama();              // Envía trama de respuesta
        flag_control_automatico = 1;  // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;              // Reestablece ID_TRAMA
        break;
      } 
      case(2):                     // Trama de control manual
      {
        make_trama(2,0);           // Prepara trama de respuesta
        send_trama();              // Envía trama de respuesta
        control_manual(inst1_man); // Va a control manual. Entra con el indicador del actuador que se prende. Sólo 1 actuador por trama se envía.  
        flag_control_automatico = 0;  // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;              // Reestablece ID_TRAMA     
        break;
      }
      case(3):                     // Trama de calibración
      {
        make_trama(3,0);           // Prepara trama de respuesta
        send_trama();              // Envía trama de respuesta
        set_calibracion(flag_cal);
        flag_control_automatico = 0;   // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático 
        id_trama = 0;                  // Reestablece ID_TRAMA
        break;
      }
      case(4):                     // Trama de config. actuadores.
      {
        make_trama(4,0);           // Prepara trama
        send_trama();              // Envía trama de respuesta
               
        /*  
        actuador_conf:        
        1: Agitador
        2: Actuador 1
        3: Actuador 2
        4: Actuador 3
        5: Bomba 1
        6: Bomba 2
        7: Bomba 3
        8: Bomba 4
        
        Valor de configuración:       
        Valor_conf_h
        Valor_conf_l  
        */ 
        
        Motor_conectar();
        
        int rpm = (valor_conf_h << 8) + (valor_conf_l);
        set_config_actuadores(actuador_conf, rpm);
      
        flag_control_automatico = 1;    // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;                   // Reestablece ID_TRAMA
        break;
      }
      case(7):                              // Trama para leer datos de calibración desde EEPROM
      {
        byte ph4_lsb = EEPROM.read(12);     // Valor de calibración almacenado para pH 4 (* 100 / 2)
        byte ph4_msb = EEPROM.read(14); 
        sensor_ph_4_cal = ((ph4_msb << 8) + ph4_lsb) * (voltaje_ref_ADC / 1023);
        byte ph7_lsb = EEPROM.read(16);                 // Valor de calibración almacenado para pH 4 (* 100 / 2)
        byte ph7_msb = EEPROM.read(18); 
        sensor_ph_7_cal = ((ph7_msb << 8) + ph7_lsb) * (voltaje_ref_ADC / 1023);
        paso_ph_cal = (float)((7 - 4) / (sensor_ph_7_cal - sensor_ph_4_cal));  // Almacena el paso y los valores de calibración en variables globales.
        ok_calibration = 1; 
        flag_control_automatico = 0;  // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;
        break;        
      }
      case(9):                             // Trama de reiniciar
      {
        make_trama(9,0);                   // Prepara trama de respuesta
        send_trama();                      // Envía trama de respuesta  
        variables_reset();                 // Resetea variables 
        actuadores_off();                  // Apaga actuadores
        flag_control_automatico = 0;       // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;                      // Reestablece ID_TRAMA  
        break;
      }
     case(10):                             // Trama de petición de data logger
     {
       make_trama(10,0);                   // Prepara trama de respuesta
       send_trama();                       // Envía trama de respuesta
       
       for(int j = 0; j <=((sector_init - 10) / 12); j++) 
       {
         make_eeprom_trama(11, j);         // Saca los datos de memoria y los pone en un arreglo
         send_trama();                     // Envía los datos del arreglo
         delay(5);                         // Tiempo de espera por cada lectura de memoria 
       }
       sector_init = 10;                   // Reestablece el sector de memoria 
       aux_eeprom = 0;
       contador_eeprom = 0;
       make_trama(12,0);                   // ID_TRAMA = 12 -> FIN EEPROM ENVÍO
       send_trama();                       // Envía la trama de FIN EEPROM
       flag_control_automatico = 0;        // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
       id_trama = 0;                       // Reestablece ID_TRAMA
       break;
     }
      case(14):                              // Trama de configuración de tasa de escritura en Datalogger.
      {
        make_trama(14,eeprom_tasa);          // Prepara trama de respuesta (envía tasa para corroborar valor recibido) 
        send_trama();                        // Envía trama de respuesta
        flag_control_automatico = 0;         // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;                        // Reestablece ID_TRAMA
        break; 
      }
      case(15):                               // Trama de Check Comunicación. Se recibe cada cierto tiempo
      {
        make_trama(15,0);                     // Prepara trama de confirmación
        send_trama();                         // Envía trama de confirmación       
        id_trama = 0;                         // Reestablece ID_TRAMA
        break; 
      }        
      default:break;
    }  // FIN SWITCH
    
    // CADA 100 ms LECTURA DE SENSORES
    timer_lectura++;        // Timer de lectura de sensores aumenta cada 100 (ms)
    lectura_sensores();     // Se llama procesar_datos() desde lectura_sensores() 
    timer_muestreo = 0;
    timer_loop = 0; 
  }    // FIN LOOP DE LECTURA CADA 100 ms
  
  if(seconds == 100)                 // Cada 1 segundo. Frecuencia de envío hacia SW
  {
    
    if (flag_control_automatico == 1)
    {
      control_automatico();
    }
    
    incrementador_tx++;              // Byte de envío. Retorna a cero despues de 255
    estado_output();                 // Estado de la salidas digitales HIGH = 1 o LOW = 0  valores almacenados en variable output_state.
    make_trama(0,incrementador_tx);  // ID_TRAMA = 0 trama normal hacia java cada 1 segundo, con incrementador_tx++
    send_trama();                    // Envía trama normal  
    
    // Contador_eeprom aumenta cada 60 segundos en la subrutina de Timer
    // Tiempo de escritura en memoria.
    if((contador_eeprom == eeprom_tasa) && (aux_eeprom == 0))  // Cada x segundos y siempre que no esté llena la memoria
    {
      EEPROM.write(sector_init,sens0_tx_h);  // inicia en el byte 10, EEPROM
      EEPROM.write(sector_init+1,sens0_tx_l);
      EEPROM.write(sector_init+2,sens1_tx_h);
      EEPROM.write(sector_init+3,sens1_tx_l);
      EEPROM.write(sector_init+4,sens2_tx_h);
      EEPROM.write(sector_init+5,sens2_tx_l);
      EEPROM.write(sector_init+6,sens3_tx_h);
      EEPROM.write(sector_init+7,sens3_tx_l);
      EEPROM.write(sector_init+8,sens4_tx_h);
      EEPROM.write(sector_init+9,sens4_tx_l);
      EEPROM.write(sector_init+10,sens5_tx_h);
      EEPROM.write(sector_init+11,sens5_tx_l);
      sector_init = sector_init + 12;
      
      make_trama(13,sector_init);              // Trama de confirmación de almacenamiento
      send_trama();
      
      if(sector_init >= 982)
      {
        sector_init = 982;
        aux_eeprom = 1;
      }
      else
      {
        aux_eeprom = 0;
      }    
      contador_eeprom = 0;
    }
    
    /****************************************************************/
    
    seconds = 0;
  }
  
  /******************************************************************/
  //  INT_TIMER  //
  if(aux_timer1 == 1)         // cada 10 (ms) 
  { 
    seconds++;               //aumenta cada 10 (ms)
    milisegundos++;          //aumenta cada 10 (ms)  
    timer_muestreo++;
    timer_loop++;      

    if(milisegundos == 100)  // milisegundos == 100 * 10 (ms) = 1 seg
    {
      milisegundos = 0;
      segundos++;
    } 
    if(segundos == 60)      // segundos == 60 = 1 minuto
    {
      segundos = 0;
      contador_eeprom++;
    }
    
    aux_timer1 = 0;    
  }
  //  END INT_TIMER  //
  /******************************************************************/
}
