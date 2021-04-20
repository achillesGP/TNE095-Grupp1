#include <AccelStepper.h>
#include <QTRSensors.h>

QTRSensors qtr;

//PID reglering
float Kp = 0.035;
float Kd = 0;
float lastError = 0;
float D;

char g;
//IR sensor, använder 7 st just nu
const uint8_t SensorCount = 9;
//Varje sensor sätts inom en array
uint16_t sensorValues[SensorCount];
bool test = false;


//Sätter max hastighet och en bas hastighet på motorn
const uint8_t rightMaxSpeed = 220;
const uint8_t leftMaxSpeed = 220;

const uint8_t rightBaseSpeed = 150;
const uint8_t leftBaseSpeed = 150;

//Sätter vilka pins motorerna använder
int enA = 9;
int MotorAip1 = 2;
int MotorAip2 = 3;
int enB = 10;
int MotorBip1 = 4;
int MotorBip2 = 5;

//Det som skickas från BT kommer förvaras i c.
char c = ' ';
char last_c = ' ';


void setup() {
  //Serial är konsolen på vår sida.
  //Serial.begin(9600);

  //Serial1 är bluetooth. 9600 säger hur snabbt information är skickad mellan varandra.
  Serial1.begin(9600);

  //Bestämmer vilka pins som ska vara outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(MotorAip1, OUTPUT);
  pinMode(MotorAip2, OUTPUT);
  pinMode(MotorBip1, OUTPUT);
  pinMode(MotorBip2, OUTPUT);


  //Allt som står under här tillhör sensorerna. De tilldelas pins och sedan kalibreras de.
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    22, 23, 24, 25, 26, 27, 28, 29
  }, SensorCount);

  delay(500);

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();

  }

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration



  digitalWrite(MotorAip1, LOW);
  digitalWrite(MotorAip2, LOW);
  digitalWrite(MotorBip1, LOW);
  digitalWrite(MotorBip2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // print the calibration minimum values measured when emitters were on




}
void loop() {
  //PID - reglering
  int position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
  int error = position - 2500; //runt position = 3700-4000 är då bilen är mitten av tejepen
  D = error - lastError; //Jämför skillnaden från senaste ändring.
  float motorSpeed = Kp * error + Kd * D; //Beräknar själva PID
  lastError = error;


  int rightMotorSpeed = rightBaseSpeed + motorSpeed; //Sätter hastigheten av motorn enligt pid beräkning
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  analogWrite(enA, rightMotorSpeed);//Högermotor hastighet
  analogWrite(enA, rightMotorSpeed);//Högermotor hastighet

  analogWrite(enB, leftMotorSpeed); //Vänstermotor hastighet

  analogWrite(enB, leftMotorSpeed); //Vänstermotor hastighet

  /* if (Serial.available()) {
     c = Serial.read();
     //Serial1.write(Serial.read());
    }*/

  if (Serial1.available()) { //Om nåt har skickats från bluetooth

    c = Serial1.read();
    Serial.println(c);

    if ((c == '0') || (c == '1') || (c == '2') || (c == '3') || (c == '4') || (c == '5') || (c == '6')) {
      Serial1.println(c);
      test = true;
    }


  }

  if (c == '8') {
    Serial1.write("System is calibration, please wait! \r\n");
    digitalWrite(MotorAip1, HIGH);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, HIGH);

    analogWrite(enA, 100);
    analogWrite(enB, 100);


    //  Serial1.write("System is done calibrating. \r\n");

  }


  //Skriver ut PID värder för fel sökning
  /*for (uint8_t i = 0; i < SensorCount; i++)
    {
    Serial.print(sensorValues[i]);
    Serial.print('\t');

    }
    Serial.println(position);*/

  /*if (position = 6500) {
    Serial.println("Can't find line");
    int counter = 0;

    while (c != '1') {
      if (Serial1.available()) {
        c = Serial1.read();
        //Serial1.write(Serial.read());
      }
      digitalWrite(MotorAip1, LOW);
      digitalWrite(MotorAip2, LOW);
      digitalWrite(MotorBip1, LOW);
      digitalWrite(MotorBip2, LOW);
      analogWrite(enA, 0);
      analogWrite(enB, 0);
    }
    while (position > 6900) {
      int position = qtr.readLineBlack(sensorValues);
      digitalWrite(MotorAip1, LOW);
      digitalWrite(MotorAip2, HIGH);
      digitalWrite(MotorBip1, LOW);
      digitalWrite(MotorBip2, HIGH);
      analogWrite(enA, 100);
      analogWrite(enB, 100);
      counter = counter + 1;
      if (position <= 6300) {
        break;
      }
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');

      }

      if (counter == 50) {
        Serial.println("Line cant be found!");
        break;
      }
      Serial.println(position);

    }


    digitalWrite(MotorAip1, LOW);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    }*/



  //Om de två sensorerna längst ut ser svart. Korsning! Stanna bilen
  if (((sensorValues[0] == 1000)  && (sensorValues[4] == 1000 )) || ((sensorValues[4] == 1000 ) && (sensorValues[7] == 1000 ))) {
    delay(150);
    if (test == true) {
      // Serial.print("Sending value ");

      Serial1.println('I');
      test = false;
    }

    digitalWrite(MotorAip1, LOW);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    //Serial.println(position);

  }

  /*if ((sensorValues[6] == 1000)  && (sensorValues[4] == 1000 )) {
    counter = counter + 1;
    if (counter > 5) {
      delay(200);

      Serial.println("Found a right crossing!!");
      digitalWrite(MotorAip1, LOW);
      digitalWrite(MotorAip2, LOW);
      digitalWrite(MotorBip1, LOW);
      digitalWrite(MotorBip2, LOW);
      analogWrite(enA, 0);
      analogWrite(enB, 0);
    }
    }*/

  //Om man skriver 0 Så ska bilen stanna
  if (c == '0')
  {
    //Serial1.write(c);

    digitalWrite(MotorAip1, LOW);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);


  }
  //Kör framåt om man skickar 1
  if (c == '1') {
    c = 1;
    //Serial1.write(c);

    digitalWrite(MotorAip1, HIGH);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, HIGH);
    digitalWrite(MotorBip2, LOW);

  }

  //Sväng höger
  if (c == '2')
  {
    c = 2;
    //  Serial1.write(c);

    int position = qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
    int counter = 0;
    while (position < 6000) {
      counter = counter + 1;

      if (counter == 1000) {
        Serial1.println('A');
        break;
      }
      if (Serial1.available()) { //Om nåt har skickats från bluetooth
        g = Serial1.read();
      }
      if (g == '0') {
        g = ' ';
        break;
      }

      //Serial.println(position);
      //Serial.println(sensorValues[6]);
      digitalWrite(MotorAip1, HIGH);
      digitalWrite(MotorAip2, LOW);
      digitalWrite(MotorBip1, LOW);
      digitalWrite(MotorBip2, HIGH);
      analogWrite (enA, 120);
      analogWrite (enB, 120);
      position = qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.

      //Serial.println(position);

    }
    digitalWrite(MotorAip1, HIGH);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, HIGH);
    digitalWrite(MotorBip2, LOW);

  }
  /*if ((sensorValues[1] == 1000) && (sensorValues[1] == 1000) && (sensorValues[2] == 1000)) {
    delay(50);
    Serial.println("Found a left crossing!!");
    digitalWrite(MotorAip1, LOW);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    }*/

  //Sväng vänster
  if (c == '3')
  {
    c = 3;
    Serial1.write(c);

    int position = qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
    int counter = 0;

    while (position > 1500) {
      counter = counter + 1;

      if (counter == 1000) {
        Serial1.println('A');
        break;
      }
      if (Serial1.available()) { //Om nåt har skickats från bluetooth
        g = Serial1.read();
      }

      if (g == '0') {
        g = ' ';
        break;
      }
      digitalWrite(MotorAip1, LOW);
      digitalWrite(MotorAip2, HIGH);
      digitalWrite(MotorBip1, HIGH);
      digitalWrite(MotorBip2, LOW);
      analogWrite (enA, 100);
      analogWrite (enB, 100);
      position = qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.

      // Serial.println(position);

    }

    digitalWrite(MotorAip1, HIGH);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, HIGH);
    digitalWrite(MotorBip2, LOW);
  }
  //Inte implementerad. Men ska backa
  if (c == '4') {
    digitalWrite(MotorAip1, LOW);
    digitalWrite(MotorAip2, HIGH);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, HIGH);
  }

  //Rensa det som har skickats.
  // c = ' ';

}













/*void left(){
    digitalWrite(MotorAip1,HIGH);
    digitalWrite(MotorAip2,LOW);
    digitalWrite(MotorBip1,LOW);
    digitalWrite(MotorBip2,HIGH);
    analogWrite (enA, 80);
    analogWrite (enB, 80);
    delay(100);
  }

  void right(){
    digitalWrite(MotorAip1,LOW);     // If I want to turn right then the speed of the right wheel should be less than that of the left wheel, here, let a be the right wheel
    digitalWrite(MotorAip2,HIGH);
    digitalWrite(MotorBip1,HIGH);
    digitalWrite(MotorBip2,LOW);
    analogWrite (enA, 80);
    analogWrite (enB, 80);
    delay(100);
  }*/
