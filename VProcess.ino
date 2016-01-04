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
int sensor_od_100_cal;

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

byte actuador_conf;    // Variable de información de actuador
byte valor_conf_h;     // Variable de valor de configuración High
byte valor_conf_l;     // Variable de valor de configuración Low
byte inst4_conf;        
byte inst5_conf;       
byte inst6_conf;    
byte inst7_conf;  
byte inst8_conf;    
byte inst9_conf;   
byte inst10_conf;    
byte inst11_conf;   
byte inst12_conf;    
byte aux1_conf;      
byte aux2_conf;     
byte aux3_conf;  
      
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

int aux_5_seg;

int ph7_init;
int ph4_init;
float paso_init;

int show_val;

float data_pt;
float data_ph;
float data_420_3;
float data_od;
float data_420_1;
float data_420_2;

float volt;
float corr;
float pendiente;

int data;
int aux;

//--------------------------------------------------------------//
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
  
  lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd_saludo();
  delay(500);
  
  flag_control_automatico = 0;
  data_cero = 0;
  
  ph7_init  = 356; // Para sensor de plástico: 394; // 490; // Equivalente al offset del circuito
  ph4_init  = 516; // Para sensor de plástico: 560; // 641;
  paso_init = -0.018;
  
  aux_5_seg = 0;
  data = 0;
  aux = 0;
  
  data_pt = 25;
  
  sens0_anterior = 0;
  sens1_anterior = 0;
  sens2_anterior = 0;
  sens3_anterior = 0;
  sens4_anterior = 0;
  sens5_anterior = 0;
}

//--------------------------------------------------------------//
// Saludo inicial en pantalla
void lcd_saludo()
{
  lcd.setCursor(6,0);
  lcd.print("VPROCESS");
  lcd.setCursor(0,1);
  lcd.print("Monitoreo y Control.");
  lcd.setCursor(5,2);
  lcd.print("Biocl S.A.");
  lcd.setCursor(0,3);
  lcd.print("____________________");
}

//--------------------------------------------------------------//
// Entra a show_val cada 5 segundos desde el loop.
void show_values()
{
  // Los valores de los actuadores se obtienen desde estado_output
  
  /*
  data_pt
  data_ph
  data_420_3
  data_od
  data_420_1
  data_420_2  
  */
  
  // actuador_D8  -> Agitador
  // actuador_D9  -> Actuador 1
  // actuador_D10 -> Actuador 2
  // actuador_D11 -> Actuador 3
  
  // PWM0  : Bomba 1
  // PWM1  : Bomba 2
  // PWM14 : Bomba 3
  // PWM15 : Bomba 4
  
  
  // estado_led   -> Agitador 
  // estado_led1  -> Actuador 1
  // estado_led2  -> Actuador 2
  // estado_led3  -> Actuador 3
  
  // estado_manual_1 -> Bomba 1
  // estado_manual_2 -> Bomba 2
  // estado_manual_3 -> Bomba 3
  // estado_manual_4 -> Bomba 4
  
  // Para show_val = 0 / 1 muestra valores de actuadores y sensores.
  if (show_val == 0)  // Entra a show_val cada 5 segundos 
  {
    lcd.clear();      // Debe limpiarse la pantalla, sino mantiene el texto en el punto que no se sobreescriba.
    
    // data_pt
    lcd.setCursor(0,0);
    lcd.print("Temp  :");
    lcd.print(data_pt);
    
    // data_ph
    lcd.setCursor(0,1);
    lcd.print("pH  :");
    lcd.print(data_ph);
    
    // data_420_3
    lcd.setCursor(0,2);
    lcd.print("4-20:");
    lcd.print(data_420_3);
    
     // data_od
    lcd.setCursor(10,0);
    lcd.print("OD");
    lcd.print(data_od);
    
    // data_420_1
    lcd.setCursor(10,1);
    lcd.print("4-20:");
    lcd.print(data_420_1);
    
    // data_420_2
    lcd.setCursor(10,2);
    lcd.print("4-20:");
    lcd.print(data_420_2);
    
    show_val = 1;
  }
  else
  {
    lcd.clear();
    
    // Agitador
    lcd.setCursor(0,0);
    lcd.print("Motor");
    lcd.print(estado_led);
    
    // Bomba 1
    lcd.setCursor(0,1);
    lcd.print("Bomb1");
    lcd.print(estado_manual_1);
    
    // Bomba 2
    lcd.setCursor(0,2);
    lcd.print("Bomb2");
    lcd.print(estado_manual_2);
    
    // Bomba 3
    lcd.setCursor(0,3);
    lcd.print("Bomb3");
    lcd.print(estado_manual_3);
    
    // Bomba 4
    lcd.setCursor(10,0);
    lcd.print("Bomb4");
    lcd.print(estado_manual_4);
    
    // ACT1
    lcd.setCursor(10,1);
    lcd.print("ACT1");
    lcd.print(estado_led1);
    
    // ACT2
    lcd.setCursor(10,2);
    lcd.print("ACT2:");
    lcd.print(estado_led2);
    
    // ACT3
    lcd.setCursor(10,3);
    lcd.print("ACT3:");
    lcd.print(estado_led3);
  
    show_val = 0;
  }
}

//--------------------------------------------------------------//
// Decodifica desde 1023 hacia valor variable. Esto para mostrar en pantalla los valores.
void deco_values(void)  
{
  // sens0 ADC0: PTC - NTC  
  // sens1 ADC1: pH
  // sens2 ADC2: 4-20 mA (3)
  // sens3 ADC3: OD
  // sens4 ADC4: 4-20 mA (1)
  // sens5 ADC5: 4-20 mA (2)
  
    // sens0 ADC0: PTC - NTC  
  float sens1_read_f = (sens0_pros * 4.85 / 1023);
  data_pt = (2.6012 * (-17.559 * sens1_read_f + 150.41) - 260.39);
//data_pt = 2,6012 * ( -15,559 * sensor_temp_value_volt + 150,41) - 269,39;
   /**************************************************/
   
  // sens1 ADC1: pH
  paso_init = ((7 - 4) / (ph7_init - ph4_init));
  data_ph = (paso_init * sens1_pros - (paso_init * ph7_init) + 7);
  /**************************************************/
  
  // sens2 ADC2: 4-20 mA (3)
  volt = (sens2_pros * 4.85 / 1023);
  corr = (volt / 220) * 1000;   // R = 220 Ohm, corriente en esa R
  //float pendiente = (200 - 0) / (20 - 4);
  pendiente = 12.5;  
  data_420_3 = pendiente * (corr - 20) + 200; 
  /**************************************************/ 
  
  // sens3 ADC3: OD

  // Hacer calibración y utilizar el valor de calibración en 100% aqui (sensor_od_100_cal)
  // 100% -> sensor_od_100_cal (valor de calibración a 100)
  // x%   -> sens3_pros (Valor actual leído y procesado)
  
  int od_porcentaje = (sens3_pros * 100) / sensor_od_100_cal;  // sensor_od_100_cal -> valor decimal del 100%
  // Falta ajuste de membrana
  
  // mg/lt en función de la temperatura a 100%
  // y = 0,0036x2 - 0,3402x + 14,408 ; y -> mg/l; x -> temperatura °C
  data_od = ((0,0036 * data_pt * data_pt) - (0,3402 * data_pt) + 14,408) * od_porcentaje;  // De acuerdo al valor de temperatura obtiene la concentración a 100%. Se multiplica por el % real.
  /**************************************************/
  
  // sens4 ADC4: 4-20 mA (1)
  volt = (sens4_pros * 4.85 / 1023);
  corr = (volt / 220) * 1000;   // R = 220 Ohm, corriente en esa R
  //float pendiente = (200 - 0) / (20 - 4);
  pendiente = 12.5;  
  data_420_1 = pendiente * (corr - 20) + 200; 
  /**************************************************/
  
  // sens5 ADC5: 4-20 mA (2)
  volt = (sens5_pros * 4.85 / 1023);
  corr = (volt / 220) * 1000;   // R = 220 Ohm, corriente en esa R
  //float pendiente = (200 - 0) / (20 - 4);
  pendiente = 12.5;  
  data_420_2 = pendiente * (corr - 20) + 200; 
  /**************************************************/
}

//--------------------------------------------------------------//
// Antes de empezar a controlar las RPM, se debe conectar al motor.
// Conecta al motor de acuerdo a indicaciones del manual.
// Entre byte y byte debe pasaar al menos 50 ms.
// El checksum sólo considera la parte baja del mismo.
// Se envía data = 0, porque si se envía 0 únicamente, la función write se indefine.
void Motor_conectar()
{
//  Serial.write("Conectar");
  delay(100); 
  Serial1.write(254);  
  delay(100); 
  Serial1.write(160);
  delay(100);
  Serial1.write(data);  // data = 0
  delay(100);
  Serial1.write(data);
  delay(100);
  Serial1.write(data);
  delay(100);
  Serial1.write(160);
}

//--------------------------------------------------------------//
// De acuerdo a instrucciones del manual, se envía el código de inicio 254, y luego los valores de RPM.
// El equipo responde con el código de respuestas, pero no se lee.
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

//--------------------------------------------------------------//
  // Se configuran las RPM del actuador s_id con su valor s_rpm.
  // Si el actuador es el agitador, entonces la información va por Serial1.
  // Si el actuador son las bombas, entonces se escribe por Salida Motor de Adafruit mapeado de 0-255.
  // A cada función de RPM se debe asigna la rpm_max, que por defecto es 1000. Esto permite mapear las RPM de 0-max a 0-255
void set_config_actuadores(byte s_id, byte s_rpm) //actuador_conf, valor_conf_h, valor_conf_l
{ 
    /*
    Actuador_conf:
    
    1: Agitador  (Se escribe por serial. Utiliza Motor RPM)
    2: Actuador 1
    3: Actuador 2
    4: Actuador 3
    5: Bomba 1   (Todas las bombas utilizan la salida de Motor a Danafruit.
    6: Bomba 2
    7: Bomba 3
    8: Bomba 4
    */
    
  // De acuerdo al actuador que se desee configurar (s_id)  
  switch(s_id)
  {
    case(1):    // Agitador
    {
      // Serial
      int rpm_h = (s_rpm >> 8) & 0xff;
      int rpm_l = s_rpm & 0xff;
      Motor_set_RPM(rpm_h,rpm_l);  // RPM High, RPM Low
      delay(500);
      Motor_set_RPM(rpm_h,rpm_l);  // RPM High, RPM Low 
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

//--------------------------------------------------------------//
// Control automático se refiere al encendido y apagado de actuadores que tienen relación con sensores del proceso.
// Para el caso de pH por ejemplo se relaciona con las bombas de ácido y base (1 y 2).
void control_automatico(void)  // Válido sólo para pH
{ 
  // sensX_pros -> Valor del sensor procesado y filtrado (decimal)
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
  
  /**************************************************************/  
  // Para Sensor PH los datos del sensor son inversos con respecto a los decimales
  // Así si el valor de pH es "menor" que cierto valor, Bomba 1 se debe encender. Para este caso, el valor de pH debiera ser "mayor" ya que se compara el decimal.
  if(sens1_pros >= sp_ph_b1_on)
  {
    AFMS.setPWM(0, 4096);     // Salida alta Bomba 1
  }
  if(sens1_pros <= sp_ph_b1_off)
  {
    AFMS.setPWM(0, 4096);     // Salida baja Bomba 1
  } 
  // Ocurre lo mismo con esta Bomba. La diferencia que se inyecta ácido, por lo que la lógica es inversa.
  if(sens1_pros <= sp_ph_b2_on)
  {
    AFMS.setPWM(1, 4096);    // Salida alta Bomba 2
  }
  if(sens1_pros >= sp_ph_b2_off)
  {
    AFMS.setPWM(1, 4096);    // Salida baja Bomba 2
  }
  /**************************************************************/  
  // sp_od_ag_on/off -> se asgina cuando se reciba una trama de control_automatico en deco_trama
  // El criterio es que se encenderá el agitador si es que el valor del sensor es menor que cierto valor. Y se apaga si es que super otro valor mayor.
  if(sens3_pros <= sp_od_ag_on)
  {
    digitalWrite(8,HIGH);    // Salida alta Agitador
  }
  if(sens3_pros >= sp_od_ag_off)
  {
    digitalWrite(8,LOW);     // Salida baja Agitador
  }  
}

//--------------------------------------------------------------//
  // Controla manualmente el funcionamiento de los actuadores
  // Hay 2 tipos de control: 
  // TIPO 1 -> Control de los relés de la tarjeta
  // TIPO 2 -> Control mediante salidas digitales 
  
void control_manual(int flag_control_m)
// flag_control_m se recibe desde deco_trama cuando la trama es manual, con el nombre de inst1_man
{
  switch(flag_control_m)
  {
    /*
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
    // Escribe por PWM un duty cicle elevado. Pero como tiene un arreglo R-C, la señal se hace continua.
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

//--------------------------------------------------------------//
  // PROCESAR DATOS //
  // ADC0: PTC - NTC 
  // ADC1: pH
  // ADC2: 4-20 mA (3)
  // ADC3: OD
  // ADC4: 4-20 mA (1)
  // ADC5: 4-20 mA (2)
  
// Filtra, almacena y procesa la variable de manera independiente, de acuerdo al filtro que se configure.
// La variable que lee el return es sens0_pros
float procesar_PT(int sensor_in)
{
  sens0_actual = 0.01 * sensor_in + sens0_anterior * 0.99;
  sens0_anterior = sens0_actual;  
  return sens0_actual;
}

//--------------------------------------------------------------//
// Filtra, almacena y procesa la variable de manera independiente, de acuerdo al filtro que se configure.
// La variable que lee el return es sens1_pros
float procesar_PH(int sensor_in)
{
  sens1_actual = 0.05 * sensor_in + sens1_anterior * 0.95;
  sens1_anterior = sens1_actual; 
  return sens1_actual; 
}

//--------------------------------------------------------------//
// Filtra, almacena y procesa la variable de manera independiente, de acuerdo al filtro que se configure.
// La variable que lee el return es sensx_pros
float procesar_420_3(int sensor_in)
{
  sens2_actual = 0.05 * sensor_in + sens2_anterior * 0.95;
  sens2_anterior = sens2_actual; 
  return sens2_actual; 
}

//--------------------------------------------------------------//
// Filtra, almacena y procesa la variable de manera independiente, de acuerdo al filtro que se configure.
// La variable que lee el return es sensx_pros
float procesar_OD(int sensor_in)
{
  sens3_actual = 0.01 * sensor_in + sens3_anterior * 0.99;
  sens3_anterior = sens3_actual;
  return sens3_actual; 
}

//--------------------------------------------------------------//
// Filtra, almacena y procesa la variable de manera independiente, de acuerdo al filtro que se configure.
// La variable que lee el return es sensx_pros
float procesar_420_1(int sensor_in)
{
  sens4_actual = 0.01 * sensor_in + sens4_anterior * 0.99;
  sens4_anterior = sens4_actual;
  return sens4_actual;  
}

//--------------------------------------------------------------//
// Filtra, almacena y procesa la variable de manera independiente, de acuerdo al filtro que se configure.
// La variable que lee el return es sensx_pros
float procesar_420_2(int sensor_in)
{
  sens5_actual = 0.01 * sensor_in + sens5_anterior * 0.99;
  sens5_anterior = sens5_actual;
  return sens5_actual;
}

//--------------------------------------------------------------//
// Apaga todos los actuadores 
void actuadores_off(void)
{
  digitalWrite(actuador_D8,LOW); 
  digitalWrite(actuador_D9,LOW);
  digitalWrite(actuador_D10,LOW);
  digitalWrite(actuador_D11,LOW);
  
  AFMS.setPWM(0, 0);
  AFMS.setPWM(1, 0);
  AFMS.setPWM(14, 0);
  AFMS.setPWM(15, 0);
}

//--------------------------------------------------------------//
// Decodifica la trama de acuerdo al valor de id_trama que se reciba.
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
      // ADC0: PTC - NTC          (sensor 1)  var_sens1 (0)
      // ADC1: pH                 (sensor 2)  var_sens2 (1)
      // ADC2: 4-20 mA (3)        (sensor 3)  var_sens3 (5)
      // ADC3: OD                 (sensor 4)  var_sens4 (2)
      // ADC4: 4-20 mA (1)        (sensor 5)  var_sens5 (3)
      // ADC5: 4-20 mA (2)        (sensor 6)  var_sens6 (4)
      
      flag_cal   = arreglo[3];  // 1: pH, 2: OD, 3: Temp. Indica el sensor a calibrar
      val_cal1   = arreglo[4];  
      val_cal2   = arreglo[5];  
      aux1_cal   = arreglo[6];  //1: pH = 4, 2: pH = 7, 3: OD = 0, 4: OD = 100, 5: Temp = 1, 6: Temp = 2 
      val_cal4   = arreglo[7];  
      val_cal5   = arreglo[8];  
      val_cal6   = arreglo[9];  
      val_cal7   = arreglo[10];  
      val_cal8   = arreglo[11]; 
      val_cal9   = arreglo[12];  
      val_cal_10 = arreglo[13];  
      val_cal_11 = arreglo[14]; 
      val_cal3   = arreglo[15];  
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

//--------------------------------------------------------------//
// Lee la trama desde Java. Si es que cumple con la estructura Sof, byte1, byte2, ..., byte17, eof Entonces devuelve 1.
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

//--------------------------------------------------------------//
// Resetea variables cuando recibe una trama de reinicio.
void variables_reset()
{

}

//--------------------------------------------------------------//
// Para calibración, almacena el valor que le corresponde de acuerdo a la constante utilizada en la decodificación.
// Los valores se almacenan aquí para registrar en memoria los datos de calibración y para mostrar en pantalla las variables en sus unidades correspondientes.
// Lo mismo hace en Java. 
byte set_calibracion(byte flag_c, byte aux1_cal_rx)
{
 // flag_cal   = arreglo[3];  // Indica el sensor a calibrar
 // aux1_cal   = arreglo[6];  // 
  
  // flag_cal:
  // ADC0: PTC - NTC          (sensor 1)  var_sens1 (0)
  // ADC1: pH                 (sensor 2)  var_sens2 (1)
  // ADC2: 4-20 mA (3)        (sensor 3)  var_sens3 (5)
  // ADC3: OD                 (sensor 4)  var_sens4 (2)
  // ADC4: 4-20 mA (1)        (sensor 5)  var_sens5 (3)
  // ADC5: 4-20 mA (2)        (sensor 6)  var_sens6 (4)
  
  // sens0_pros -> ADC0: PTC - NTC 
  // sens1_pros -> ADC1: pH
  // sens2_pros -> ADC2: 4-20 mA (3)
  // sens3_pros -> ADC3: OD
  // sens4_pros -> ADC4: 4-20 mA (1)
  // sens5_pros -> ADC5: 4-20 mA (2)
  
  // aux1_cal: 
  // 0 -> Primer valor de calibración 
  // 1 -> Segundo valor de calibración
  
        
  // Calibra en función de la variable de interés y del dato aux1_cal que determina el valor 1 o 2 de calirbación.
  switch(flag_c)
  {
    case(0):  // PTC
    {
      break;
    }
    case(1):  // PH
    { 
      if(aux1_cal_rx == 1) //calibracion pH7
      {
        ph7_init = sens1_pros; // Lee el valor actual decimal filtrado
        
        // SET CAL VAL ON EEMPROM
        int ph7_dec = sens1_pros; //0 a 1023
        byte ph7_dec_lsb = 0xFF & ph7_dec;
        byte ph7_dec_msb = 0xFF & (ph7_dec >> 8);
        EEPROM.write(16,ph7_dec_lsb);
        EEPROM.write(18,ph7_dec_msb);
        
        flag_ph1 = 1;
      }
      else if(aux1_cal_rx == 2) //calibracion con pH4
      {
        ph4_init = sens1_pros; // Lee el valor actual decimal filtrado    

        // SET CAL VAL ON EEMPROM
        int ph4_dec = sens1_pros; //0 a 1023
        byte ph4_dec_lsb = 0xFF & ph4_dec;
        byte ph4_dec_msb = 0xFF & (ph4_dec >> 8);
        EEPROM.write(12,ph4_dec_lsb);
        EEPROM.write(14,ph4_dec_msb);    

        flag_ph2 = 1;
      }
      
      if((flag_ph1 == 1) && (flag_ph2 == 1))
      {
        paso_init = ((7 - 4) / (ph7_init - ph4_init)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1 = 0;
        flag_ph2 = 0;
      }
      
      break;
    }
    case(2):  // OD
    { 
      if(aux1_cal_rx == 1) //calibracion OD 100%
      {
        sensor_od_100_cal = sens3_pros; // Lee el valor actual decimal filtrado del 100% de oxígeno
        
        // SET CAL VAL ON EEMPROM
        int od100_dec = sens3_pros; //0 a 1023
        byte od100_dec_lsb = 0xFF & od100_dec;
        byte od100_dec_msb = 0xFF & (od100_dec >> 8);
        EEPROM.write(16,od100_dec_lsb);
        EEPROM.write(18,od100_dec_msb);
        
        // flag_od1 = 1;
      }
      break;
    }
    case(3):  // 4-20 mA (1)
    { 
      if(aux1_cal_rx == 1) //calibracion pH4
      {
        controlador_ph4_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph1_cont = 1;
      }
      else if(aux1_cal_rx == 2) //calibracion con pH7
      {
        controlador_ph7_cal = controlador_ph_value; //lectura pH4 de bit a milivolts
        flag_ph2_cont = 1;
      }     
      if((flag_ph1_cont == 1) && (flag_ph2_cont == 1))
      {
        paso_ph_cont = ((7 - 4) / (controlador_ph7_cal - controlador_ph4_cal)); // pH/Volt paso para cotrolador y adapatdor de 4-20 mA, voltaje referencia ADC igual a 4.85 Volts
        flag_ph1_cont = 0;
        flag_ph2_cont = 0;
      }
      break;     
    }
    case(4):  // 4-20 mA (2)
    { 
      break;
    }
    case(5):  // 4-20 mA (3)
    { 
      break;
    }
    default:break;
  }
} 

//--------------------------------------------------------------//
// La lectura de sensores ocurre de manera colaborativa en el tiempo, dividiendo la frecuencia de oscilación en tantas lecturas se deban ejecutar.
void lectura_sensores()
{ 
  // sens0_pros -> ADC0: PTC - NTC 
  // sens1_pros -> ADC1: pH
  // sens2_pros -> ADC2: 4-20 mA (3)
  // sens3_pros -> ADC3: OD
  // sens4_pros -> ADC4: 4-20 mA (1)
  // sens5_pros -> ADC5: 4-20 mA (2)

  switch(timer_lectura)             // Lectura de un sensor cada 0.1 (s), equivale a un tiempo de lectura = 0.6 (s) x sensor,  fs= (1/0.6)
  { 
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
      sens1_pros = procesar_PH(sens1_read); // 60 ms, entra decimal, sale flotante 
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

//--------------------------------------------------------------//
// Estado de los Rele - salidas digitales
void estado_output(void)  
{
  
  // actuador_D8  -> Agitador
  // actuador_D9  -> Actuador 1
  // actuador_D10 -> Actuador 2
  // actuador_D11 -> Actuador 3
  
  // PWM0  : Bomba 1
  // PWM1  : Bomba 2
  // PWM14 : Bomba 3
  // PWM15 : Bomba 4
  
  
  // estado_led   -> Agitador 
  // estado_led1  -> Actuador 1
  // estado_led2  -> Actuador 2
  // estado_led3  -> Actuador 3
  
  // estado_manual_1 -> Bomba 1
  // estado_manual_2 -> Bomba 2
  // estado_manual_3 -> Bomba 3
  // estado_manual_4 -> Bomba 4
  
  // bitWrite(x, n, b)
  // x: the numeric variable to which to write
  // n: which bit of the number to write, starting at 0 for the least-significant (rightmost) bit
  // b: the value to write to the bit (0 or 1)
  
  output_state = 0;
  
  estado_led  = digitalRead(actuador_D8);  // Lee 0 - 1 en función del estado digital.
  bitWrite(output_state, 0, estado_led);   // Escribe en el bit 0 de output_state el valor digital estado_led
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

//--------------------------------------------------------------//
// Toma cada uno de los valores leídos y filtrados, los divide en parte alta y baja. Los envía de ese modo.
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

//--------------------------------------------------------------//
// Lee y prepara la trama desde la EEPROM para ser enviada cuando se solicite.
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

//--------------------------------------------------------------//
// Envía la trama por serial
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

//--------------------------------------------------------------//
// Interrupción de timer de acuerdo a la configuración de setup.
ISR(TIMER1_COMPA_vect)   //Flag correspondiente a timer1 comparacion
{                        
    aux_timer1 = 1;     
}

//--------------------------------------------------------------//
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
      // Cada trama de 19 bytes contiene un byte id_trama, que identifica la trama de acuerdo a la función de ésta.
      // Cada vez que se recibe una trama se prepara una nueva trama con datos = 0, excepto el id_trama que será igual al recibido.
      // .. De este modo, se confirma la recepción del envío en el SW.
      // Toda la lectura de la trama y decodificación de valores se hace en la subrutina de Serial.available().
      
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
        set_calibracion(flag_cal,aux1_cal);  // flag_cal indica el sensor que se calibrará. 1: pH, 2: OD, 3: Temp. Indica el sensor a calibrar. Se lee desde la trama recibida.
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
        
        Motor_conectar();    // Envía señal para iniciar comunicación con Motor. No se lee la respuesta
        
        int rpm = (valor_conf_h << 8) + (valor_conf_l);  // Bytes que se reciben en la trama y que corresponden al valor en RPM 
        set_config_actuadores(actuador_conf, rpm);       // Se configuran las RPM del actuador en función del actuador que se requiera y de la rpm transmitida.
      
        flag_control_automatico = 1;    // Flag de control automático. Será 0 para interrumpir el proceso de control. Será 1 cuando reciba trama de control_automático
        id_trama = 0;                   // Reestablece ID_TRAMA
        break;
      }
      case(7):                              // Trama para leer datos de calibración desde EEPROM
      {
        byte ph4_lsb = EEPROM.read(12);     // Valor de calibración almacenado para pH 4 (* 100 / 2)
        byte ph4_msb = EEPROM.read(14); 
        ph4_init = ((ph4_msb << 8) + ph4_lsb) * (voltaje_ref_ADC / 1023);
        byte ph7_lsb = EEPROM.read(16);                 // Valor de calibración almacenado para pH 4 (* 100 / 2)
        byte ph7_msb = EEPROM.read(18); 
        ph7_init = ((ph7_msb << 8) + ph7_lsb) * (voltaje_ref_ADC / 1023);
        paso_init = (float)((7 - 4) / (ph7_init - ph4_init));  // Almacena el paso y los valores de calibración en variables globales.
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
    aux_5_seg++;                     // Flag para mostrar datos en pantalla, Cada 5 seg refresca
    
    if (flag_control_automatico == 1)  // Flag de control automatico se fija cuando la trama es de control automatico (id_trama = 1). Aquí se llama a la subrutina de control
    {
      control_automatico();            // Llamada de subrutina de control automatico. Se procesa siempre que flag_control_automatico == 1
    }
    
    incrementador_tx++;              // Byte de envío. Retorna a cero despues de 255
    estado_output();                 // Estado de la salidas digitales HIGH = 1 o LOW = 0  valores almacenados en variable output_state.
       
    if (aux_5_seg == 5)
    {
      deco_values();
      show_values();
      aux_5_seg = 0;
    }
    
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
