/*
Comandos git:
  git commit -a -m "primero test"
  git push --set-upstream origin master
*/

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "Wire.h"
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0    // 32768/2
#define G_R 131.0       // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//Declaracion objeto otro nucleo
TaskHandle_t Task1;

//Declaracion objeto Bluetoodth
BluetoothSerial SerialBT;

#define led 2

//Pines motor
#define Derecho_A 34
#define Derecho_B 35
#define Derecho_E 13
#define Derecho_Dir_1 26
#define Derecho_Dir_2 27
#define Izquierdo_A 23
#define Izquierdo_B 22
#define Izquierdo_E 25
#define Izquierdo_Dir_1 14
#define Izquierdo_Dir_2 12

int Adelante [2] = {0,1};
int Atras [2] = {1,0};
int Quieto [2] = {0,0};
long lastAction = 0;
int count = 0;

//Variables de comunicacion
String accion = "";
String incoming = "";
String to_send = "";
bool in = false;

// setting PWM properties
const int freq = 1000;
const int ChannelI = 0;
const int ChannelD = 1;
const int resolution = 10;

//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[3];
float Gy[3];
float Angle[3];
 
String valores;
 
long tiempo_prev;
float dt;

//Declaracion de funciones
void codeForTask1(void *parameter);
void mover(int Dir_D[2], int Dir_I[2], int PWM_D, int PWM_I);
void adelante();
void atras();
void derecha();
void isquierda();
void quieto();

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    codeForTask1, /* Task function. */
    "Task_1",  /* name of task. */
    1000,     /* Stack size of task */
    NULL,     /* parameter of the task */
    1,        /* priority of the task */
    &Task1,     /* Task handle to keep track of created task */
    0);       /* Core */
  
  SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(led, OUTPUT);
  
  pinMode(Derecho_A, INPUT);
  pinMode(Derecho_B, INPUT);
  pinMode(Derecho_Dir_1, OUTPUT);
  pinMode(Derecho_Dir_2, OUTPUT);
  pinMode(Izquierdo_A, INPUT);
  pinMode(Izquierdo_B, INPUT);
  pinMode(Izquierdo_Dir_1, OUTPUT);
  pinMode(Izquierdo_Dir_2, OUTPUT);

  
  // configure LED PWM functionalitites
  ledcSetup(ChannelI, freq, resolution);
  ledcSetup(ChannelD, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Izquierdo_E, ChannelI);
  ledcAttachPin(Derecho_E, ChannelD);
}

void loop() {
   long now = millis();
   if (now - lastAction > 10){
     //Leer los valores del Acelerometro de la IMU
     Wire.beginTransmission(MPU);
     Wire.write(0x3F); //Pedir el registro 0x3F - corresponde al AcZ
     Wire.endTransmission(false);
     Wire.requestFrom(MPU,2,true);   //A partir del 0x3B, se piden 6 registros
     AcZ=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   
     //Leer los valores del Giroscopio
     Wire.beginTransmission(MPU);
     Wire.write(0x47);
     Wire.endTransmission(false);
     Wire.requestFrom(MPU,2,true);   //A partir del 0x43, se piden 6 registros
     GyZ=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   
     //Calculo del angulo del Giroscopio
     Gy[2] = GyZ/G_R;
   
     dt = (millis() - tiempo_prev) / 1000.0;
     tiempo_prev = millis();
   
     //Integración respecto del tiempo paras calcular el YAW
     Angle[2] = Angle[2]+Gy[2]*dt-0.00196; //Este 0.012 es para el ERROR lo Puso Sr. Vega
   
     //Mostrar los valores por consola
     valores = "90, " + String(Angle[2]) + ", -90";
     Serial.println(valores);
   }
   
   if (now - lastAction > 100){
     lastAction = now;
     if(count<4){
        quieto();
     }else{
       if(accion=="B"){
          atras();
       }
       if(accion=="R"){
          derecha();
       }
       if(accion=="L"){
          izquierda();
       }
       if(accion=="F"){
          adelante();
       }
       if(accion=="S"){
          quieto();
       }
       if(accion=="W"){
          to_send = ""+String(Angle[2])+" "+String(tiempo_prev)+"";//'1';
       }
       if(accion=="w"){
          to_send = '0';
       }
       accion="";
     }
     count++;
  }
  
  if(in){
    if(incoming == "W"){
      digitalWrite(led,HIGH);
    }
    if(incoming == "w"){
      digitalWrite(led,LOW);
    }
    accion=incoming;
    incoming ="";
    in = false;
  }
  
}

//*****************************
//***   TAREA OTRO NUCLEO   ***
//*****************************

void codeForTask1(void *parameter)
{

  for (;;)
  {
    
    while (SerialBT.available()) {
      int letra = SerialBT.read();
      incoming += (char)letra;
    }

    if(incoming != ""){
      Serial.println(incoming);
      in = true;
    }

    if(to_send != ""){
      char msg[25];
      char voidchar;
      to_send.toCharArray(msg, 100);
      int i=0;
      while(msg[i] != voidchar){
        Serial.print(msg[i]);
        SerialBT.write(msg[i]);
        i++;
      }
      Serial.println();
      to_send="";
    }

    vTaskDelay(20);
  }
}

//*****************************
//*******   FUNCIONES   *******
//*****************************

void mover(int Dir_D[2], int Dir_I[2], int PWM_D, int PWM_I){
  digitalWrite(Derecho_Dir_1, Dir_D[0]);
  digitalWrite(Derecho_Dir_2, Dir_D[1]);
  digitalWrite(Izquierdo_Dir_1, Dir_I[0]);
  digitalWrite(Izquierdo_Dir_2, Dir_I[1]);
  ledcWrite(ChannelD, PWM_D); //Salida analogica Derecha
  ledcWrite(ChannelI, PWM_I); //Salida analogica Isquierda
}

void adelante(){
  mover(Adelante, Adelante, 512, 512);
}

void atras(){
  mover(Atras, Atras, 512, 512);
}

void derecha(){
  mover(Adelante, Adelante, 256, 512);
}

void izquierda(){
  mover(Adelante, Adelante, 512, 256);
}

void quieto(){
  mover(Quieto, Quieto, 0, 0);
}
