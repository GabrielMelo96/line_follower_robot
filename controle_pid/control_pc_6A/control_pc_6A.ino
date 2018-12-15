#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install
#include <QTRSensors.h>

  
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog QTRA((unsigned char[]) {8,9,10,11,12,13},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
 
//pwm 
#define PWMMIN  0  //50
#define PWMMAX 140  //150
#define Kp 200.0// var angular de 0 - 2123
#define Kd 20.0
#define Ki 0.0
double Reduction=0.5;
double THRESHMARK = 500;
 
//Maina
bool debugSen = false;
bool debugMotor = false;
bool debugMark = false;

//INF
#define INF 0xffffffff

//motor
typedef struct MotorsS {
  int pwmL;
  int pwmR;
} Motors;

Motors M;

AF_DCMotor motorEsq(4, MOTOR12_64KHZ);
AF_DCMotor motorDir(3, MOTOR12_64KHZ);

//--------------------------------Dimensões do robô
//dimensões do robô
#define COMP 0.1316 //1316
#define EIXO 0.1000 //1379
#define RAIO 0.0213 //0.0333

//--------------------------------Distâncias dos sensores
//Peso dos sensores
//#define DS1 -0.03375
#define DS1 -0.024
#define DS2 -0.0145
#define DS3 -0.0045
#define DS4 0.0045
#define DS5 0.0145
#define DS6 0.024
//#define DS8 0.03375

//--------------------------------Pinos dos Sensores
//Pinos dos sensores da centroid
#define S1PIN A8
#define S2PIN A9
#define S3PIN A10
#define S4PIN A11
#define S5PIN A12
#define S6PIN A13
//#define S7PIN A14
//#define S8PIN A15

//Pino para marcas
#define SMARK A7

//pino para curvas


//--------------------------------Leitura dos sensores
//valor máximo de leitura do sensor /10
#define READMAX 1024
#define CUTMARK 500
unsigned int sensRead[6];
unsigned int oldsensRead[6];
unsigned int sensMark,sensCurve;
int j;

void readSens(){
  unsigned int position = QTRA.readLine(sensorValues);
   
  for (j=0;j<6;j++){
          oldsensRead[j]=sensRead[j];
  }
  sensRead[0]=READMAX-sensorValues[0];
  sensRead[1]=READMAX-sensorValues[1];
  sensRead[2]=READMAX-sensorValues[2];
  sensRead[3]=READMAX-sensorValues[3];
  sensRead[4]=READMAX-sensorValues[4];
  sensRead[5]=READMAX-sensorValues[5];
  
  sensMark=READMAX-analogRead(SMARK);

  if(sensRead[0]<CUTMARK && sensRead[1]<CUTMARK && sensRead[2]<CUTMARK && sensRead[3]<CUTMARK &&
      sensRead[4]<CUTMARK && sensRead[5]<CUTMARK ){
        for (j=0;j<6;j++){
          sensRead[j]=oldsensRead[j];
        }
        
      }
  
/*
  sensRead[0]=READMAX-analogRead(S1PIN);
  sensRead[1]=READMAX-analogRead(S2PIN);
  sensRead[2]=READMAX-analogRead(S3PIN);
  sensRead[3]=READMAX-analogRead(S4PIN);
  sensRead[4]=READMAX-analogRead(S5PIN);
  sensRead[5]=READMAX-analogRead(S6PIN);
  
// 
  sensMark=READMAX-analogRead(SMARK);
  */
  if(debugSen){
    Serial.print(" "); Serial.print(sensRead[0]);
    Serial.print(" "); Serial.print(sensRead[1]);
    Serial.print(" "); Serial.print(sensRead[2]);
    Serial.print(" "); Serial.print(sensRead[3]);
    Serial.print(" "); Serial.print(sensRead[4]);
    Serial.print(" "); Serial.print(sensRead[5]);
    Serial.print("   Mark:"); Serial.print(sensMark);
  }

}

//--------------------------------Rotina de calibração dos sensores
//valor de corte das marcas
//#define MARKC 10
//#define THRESMARK 6
double MARKC;



//--------------------------------Calculo da centroid
double WSens[6] = {DS1, DS2, DS3, DS4, DS5, DS6};
double err;

void centroid(){
  int i;
  double sum = 0.0, sumW = 0.0;

  readSens();

  for (i = 0; i < 6; i++) {
    sumW += (double)sensRead[i] * WSens[i];
    sum += (double)sensRead[i];
  }

  err = sumW / sum;
  //err = sumW / (sum/8);
}

//--------------------------------Detecção de marcas
//tempo entre marcas
#define TBMARKS 300
//controle de tempo entre marcas
unsigned long tmark;
//estado
int state;

void detectaMarcas(){
  if((tmark == INF)){
    if(debugMark){
      Serial.print(state); 
      Serial.print(sensMark); Serial.print(" ");
      Serial.println(THRESHMARK);
    }
    switch(state) {
      case 4:
        delay(200);
        motorEsq.setSpeed(0);
        motorDir.setSpeed(0);
        motorEsq.run(RELEASE);
        motorDir.run(RELEASE);
        delay(200000);
        break;
      default:
        if (sensMark >= THRESHMARK) {
          state++;
          tmark = millis();
        }
    }
  } else if(tmark != INF){
    if ((millis() - tmark) >= TBMARKS) {
      tmark = INF;
    }
  }
}

//--------------------------------Control


//velocidade angular máxima
//#define Wmax 21.2622
#define Wmax 6.0

//tempo de amostragem
#define T 2  //

double Wk;
double Vk;
double Wr;
double Wl;
double deltaWRoda;
double eixoRaio = EIXO / RAIO;

//tempo
unsigned long lt, var;

double err_ante = 0, P = 0.0, I = 0.0, D = 0.0;
double a = ((PWMMAX - PWMMIN)/Wmax);

void control() {
  P = Kp * err;
  I += Ki * err *(T*0.001);
  D = Kd * (err - err_ante) / (T * 0.001);
  Wk = P + I + D;

  deltaWRoda = eixoRaio * Wk;
  Vk = ((2.0 * Wmax - abs(deltaWRoda)) / 2.0) * RAIO;
  Wl = ((2.0 * Vk - Wk * EIXO) / (2.0 * RAIO));
  Wr = ((2.0 * Vk + Wk * EIXO) / (2.0 * RAIO));

  M.pwmL = a*Wl;
  M.pwmR = a*Wr;
}


void setup() {
  sensRead[0]=0;sensRead[1]=0;sensRead[2]=700;sensRead[3]=700;sensRead[4]=0;sensRead[5]=0;
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    QTRA.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  } 
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
  //delay(5000);
  state=0;
  tmark=INF;
  lt = var = millis();
  
 
}


void loop() {
  if ((millis() - lt) >  T ) {
    lt = millis();

    //if (millis() - var > 1000 * 10){
     // motorDir.setSpeed(0);
     //motorEsq.setSpeed(0);
    //}else{
    centroid();
    //detectaMarcas();
    detectaMarcas();
    control();
    
    motorEsq.setSpeed(constrain(PWMMIN + abs(M.pwmL), PWMMIN, PWMMAX));
    
    if (M.pwmL < 0) {
      //motorEsq.setSpeed(PWMMIN - M.pwmL);
      motorEsq.run(BACKWARD);
    } else {
      //motorEsq.setSpeed(PWMMIN + M.pwmL);
      motorEsq.run(FORWARD);
    }

    motorDir.setSpeed(constrain(PWMMIN + abs(M.pwmR), PWMMIN, PWMMAX));
    
    if (M.pwmR < 0) {
      //motorDir.setSpeed(PWMMIN - M.pwmR);
      motorDir.run(BACKWARD);
    } else {
      //motorDir.setSpeed(PWMMIN + M.pwmR);
      motorDir.run(FORWARD);
    }
    err_ante = err;
     
    if(debugSen || debugMotor){
     Serial.print("\terr: "); Serial.print(err, 4);
    }
    
    if(debugMotor){
      Serial.print("\t Wl: "); Serial.print(Wl);
      Serial.print(" Wr: "); Serial.print(Wr);

      Serial.print("\t ML_pwm: "); Serial.print(M.pwmL);
      Serial.print(" MR_pwm: "); Serial.print(M.pwmR);

      Serial.print("\t PWM_L_real: "); Serial.print(constrain(PWMMIN + abs(M.pwmL), PWMMIN, PWMMAX));
      Serial.print(" PWM_R_real: "); Serial.print(constrain(PWMMIN + abs(M.pwmR), PWMMIN, PWMMAX));
    }
    
    if(debugSen || debugMotor){
      Serial.println(" ");
    }

    }
  
}
