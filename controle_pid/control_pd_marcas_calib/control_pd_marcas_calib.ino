#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install

//pwm 
#define PWMMIN  0  //50
#define PWMMAX 70  //150
#define Kp 250.0// var angular de 0 - 2123
#define Kd 35.0
#define Ki 2.0
double THRESHMARK = 150;

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

AF_DCMotor motorEsq(3, MOTOR12_64KHZ);
AF_DCMotor motorDir(4, MOTOR12_64KHZ);

//--------------------------------Dimensões do robô
//dimensões do robô
#define COMP 0.1316 //1316
#define EIXO 0.1000 //1379
#define RAIO 0.0213 //0.0333

//--------------------------------Distâncias dos sensores
//Peso dos sensores
#define DS1 -0.03375
#define DS2 -0.024
#define DS3 -0.0145
#define DS4 -0.0045
#define DS5 0.0045
#define DS6 0.0145
#define DS7 0.024
#define DS8 0.03375

//--------------------------------Pinos dos Sensores
//Pinos dos sensores da centroid
#define S1PIN A8
#define S2PIN A9
#define S3PIN A10
#define S4PIN A11
#define S5PIN A12
#define S6PIN A13
#define S7PIN A14
#define S8PIN A15

//Pino para marcas
#define SMARK A7

//pino para curvas


//--------------------------------Leitura dos sensores
//valor máximo de leitura do sensor /10
#define READMAX 1024
unsigned int sensRead[8];
unsigned int sensMark,sensCurve;

void readSens(){
  
  sensRead[0]=READMAX-analogRead(S1PIN);
  sensRead[1]=READMAX-analogRead(S2PIN);
  sensRead[2]=READMAX-analogRead(S3PIN);
  sensRead[3]=READMAX-analogRead(S4PIN);
  sensRead[4]=READMAX-analogRead(S5PIN);
  sensRead[5]=READMAX-analogRead(S6PIN);
  sensRead[6]=READMAX-analogRead(S7PIN);
  sensRead[7]=READMAX-analogRead(S8PIN);
  sensMark=READMAX-analogRead(SMARK);
  
  if(debugSen){
    Serial.print(" "); Serial.print(sensRead[0]);
    Serial.print(" "); Serial.print(sensRead[1]);
    Serial.print(" "); Serial.print(sensRead[2]);
    Serial.print(" "); Serial.print(sensRead[3]);
    Serial.print(" "); Serial.print(sensRead[4]);
    Serial.print(" "); Serial.print(sensRead[5]);
    Serial.print(" "); Serial.print(sensRead[6]);
    Serial.print(" "); Serial.print(sensRead[7]);
    Serial.print("   Mark:"); Serial.print(sensMark);
  }

}

//--------------------------------Rotina de calibração dos sensores
//valor de corte das marcas
//#define MARKC 10
//#define THRESMARK 6
double MARKC;



//--------------------------------Calculo da centroid
double WSens[8] = {DS1, DS2, DS3, DS4, DS5, DS6, DS7, DS8};
double err;

void centroid(){
  int i;
  double sum = 0.0, sumW = 0.0;

  readSens();

  for (i = 0; i < 8; i++) {
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
      Serial.print(state); Serial.print(" ");
      Serial.print(sensMark); Serial.print(" ");
      Serial.println(THRESHMARK);
    }
    switch(state) {
      case 3:
        delay(500);
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
#define T 5

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
  //calibrate();
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
