#include <AccelStepper.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

Servo Claw;
QTRSensors qtr;

const uint8_t rightMaxSpeed = 180;
const uint8_t leftMaxSpeed = 180;
long previousMillis = 0;
long interval = 5000;
long powerpreviousMillis = 0;
long powerinterval = 1000;

const uint8_t rightBaseSpeed = 150;
const uint8_t leftBaseSpeed = 150;
const uint8_t turnSpeed = 125;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//Sätter vilka pins motorerna använder
const int enA = 9;
const int MotorAip1 = 2;
const int MotorAip2 = 3;
const int enB = 10;
const int MotorBip1 = 4;
const int MotorBip2 = 5;

const int Warning_LED = 53;
const int Bluetooth_LED = 51;
const int POW_R_LED = 49;
const int POW_Y2_LED = 45;
const int POW_Y1_LED = 47;
const int POW_G2_LED = 43;
const int POW_G1_LED = 41;
const int F_LED = 31;
const int L_LED = 37;
const int R_LED = 35;
const int B_LED = 39;
const int Idle_LED = 33;
const int pos = 10;
const int claw_confirm = 13;
const int claw_output = 11;


char c = ' ';
char g = ' ';

int counters = 0;
bool increase_pid = false;

int test_counter = 0;
int error_counter = 0;

float Kp = 0.045;
float Kd = 0.025;
float Ki = 0;
float P = 0;
float I = 0;
float D = 0;
float lastError = 0;

bool counting_Crossings = false;
bool initiation = false;
bool test = false;
bool crossing_test = false;
bool lost_tape_check = false;
int lost_tape = 0;

long power = 0;
char last_turn = ' ';
int timer_check = 10000;

void setup() {
  Serial.begin(9600);  //Serial är konsolen på vår sida.
  Serial2.begin(9600); //Serial2 är bluetooth. 9600 säger hur snabbt information är skickad mellan varandra.

  //Bestämmer vilka pins som ska vara outputs
  Claw.attach(12);
  Claw.write(5);
  Claw.detach();

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(MotorAip1, OUTPUT);
  pinMode(MotorAip2, OUTPUT);
  pinMode(MotorBip1, OUTPUT);
  pinMode(MotorBip2, OUTPUT);

  pinMode(claw_confirm, INPUT_PULLUP);
  pinMode(claw_output, OUTPUT);
  pinMode(Bluetooth_LED, OUTPUT);
  pinMode(Warning_LED, OUTPUT);
  pinMode(Idle_LED, OUTPUT);
  pinMode(POW_R_LED, OUTPUT);
  pinMode(POW_Y2_LED, OUTPUT);
  pinMode(POW_Y1_LED, OUTPUT);
  pinMode(POW_G2_LED, OUTPUT);
  pinMode(POW_G1_LED, OUTPUT);
  pinMode(F_LED, OUTPUT);
  pinMode(L_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  digitalWrite(Idle_LED, LOW);
  digitalWrite(POW_R_LED, LOW);
  digitalWrite(POW_Y2_LED, LOW);
  digitalWrite(POW_Y1_LED, LOW);
  digitalWrite(POW_G2_LED, LOW);
  digitalWrite(POW_G1_LED, LOW);
  digitalWrite(claw_output, HIGH);


  //Allt som står under här tillhör sensorerna. De tilldelas pins och sedan kalibreras de.
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    28, 26, 24, 22, 29, 27, 25, 23
  }, SensorCount);

  delay(500);
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
  } else {
    Serial.println("Found INA260 chip");
  }

  Serial.println("Calibrating...");
  digitalWrite(F_LED, HIGH);
  digitalWrite(R_LED, HIGH);
  digitalWrite(L_LED, HIGH);
  digitalWrite(B_LED, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();

  }
  digitalWrite(F_LED, LOW);
  digitalWrite(R_LED, LOW);
  digitalWrite(L_LED, LOW);
  digitalWrite(B_LED, LOW);
  Serial.println("Done Calibrating...");

  digitalWrite(MotorAip1, LOW);
  digitalWrite(MotorAip2, LOW);
  digitalWrite(MotorBip1, LOW);
  digitalWrite(MotorBip2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}


void loop() {
  lost_tape_check = false;
  unsigned long powercurrentMillis = millis();
  if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

    powerpreviousMillis = powercurrentMillis;
    power = ina260.readPower() + power;
    check_power();

  }
  if (Serial2.available()) { //Om nåt har skickats från bluetooth
    c = Serial2.read();
    Serial.println(c);
    if (isAlphaNumeric(c)) {
      if ((c == '0') || (c == '1') || (c == '2') || (c == '3') || (c == '4') || (c == '5') || (c == '6') ) {
        Serial2.println(c);
        Serial.print("Sending: "); Serial.println(c);
        if (c == '2' || c == '3') {
          last_turn = c;
        }
        test = true;
        if ((c == '1') || (c == '2') || (c == '3') ||  (c == '4')) {
          counting_Crossings = true;
          Serial.print("counting Crossings: ");
          initiation = true;
        }
      } else  {
        Serial2.println(c);
        Serial2.println('C');
      }
    }

  }
  int position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
  int error = position - 3500; //runt position = 3200-3500 är då bilen är mitten av tejepen
  D = error - lastError; //Jämför skillnaden från senaste ändring.
  int I = I + error;
  float motorSpeed = Kp * error + Kd * D + Ki * I; //Beräknar själva PID
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed - motorSpeed; //Sätter hastigheten av motorn enligt pid beräkning
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;
  analogWrite(enA, rightMotorSpeed); //Högermotor hastighet
  analogWrite(enB, leftMotorSpeed ); //Vänstermotor hastighet


  if (increase_pid == true) {
    test_counter = test_counter + 1;

  }
  if (increase_pid == false) {
    test_counter = 0;
  }

  if (test_counter >= 200) {
    test_counter = 0;
    analogWrite (enA, 180);
    analogWrite (enB, 180);
    delay(50);
  }



  if (counting_Crossings == true) {
    counters = counters + 1;

  }

  if (counters >= 10) {
    counting_Crossings = false;
    counters = 0;
  }
  if (counting_Crossings == false) {
    if (((sensorValues[5] == 1000  && sensorValues[7] == 1000)) || ((sensorValues[4] == 1000  && sensorValues[6] == 1000))) {
      analogWrite (enA, 120);
      analogWrite (enB, 130);
      delay(150);
      stop_motor();
      initiation = false;
      increase_pid = false;
      turn_off_LEDS();

      if (test == true) {
        Serial2.println('I');
        digitalWrite(Idle_LED, HIGH);

        test = false;
      }
      test_counter = 0;
    }
    if (((sensorValues[0] == 1000)  && (sensorValues[2] == 1000)) || ((sensorValues[1] == 1000)  && (sensorValues[3] == 1000))) {
      analogWrite (enA, 120);
      analogWrite (enB, 130);
      delay(150);
      stop_motor();
      initiation = false;
      increase_pid = false;
      turn_off_LEDS();

      if (test == true) {
        Serial2.println('I');
        digitalWrite(Idle_LED, HIGH);

        test = false;
      }
      test_counter = 0;
    }

  }

  //Skriver ut PID värder för fel sökning
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);


  //Om man skriver 0 Så ska bilen stanna
  if (c == '0')
  {
    turn_off_LEDS();
    initiation = true;
    increase_pid = false;
    Serial2.println(power / 1000);

    c = 0;
    stop_motor();
    delay(100);
  }

  //Kör framåt om man skickar 1
  if (c == '1') {
    turn_off_LEDS();
    c = 1;
    digitalWrite(F_LED, HIGH);
    forward();

  }

  //Sväng höger
  if (c == '2')
  {
    lost_tape_check = false;

    c = 2;
    turn_off_LEDS();
    digitalWrite(R_LED, HIGH);
    qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
    unsigned long currentMillis = millis();

    previousMillis = currentMillis;
    while (sensorValues[7] != 1000) {

      qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
      right();
      analogWrite (enA, turnSpeed);
      analogWrite (enB, turnSpeed);
      unsigned long currentMillis = millis();
      unsigned long powercurrentMillis = millis();

      if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

        powerpreviousMillis = powercurrentMillis;
        power = ina260.readPower() + power;
        check_power();

      }
      if (currentMillis - previousMillis >= interval) {
        lost_tape_check = true;
        previousMillis = currentMillis;
        break;
      }
      test_counter = test_counter + 1;
      if (test_counter >= 200) {
        test_counter = 0;
        analogWrite (enA, 250);
        analogWrite (enB, 250);
        delay(50);
      }
    }

    stop_motor();
    lost_tape = 0;
    test_counter = 0;
    delay(500);
    qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
    if (lost_tape_check == true) {
      Serial2.println('A');
      stop_motor();

    } else {
      unsigned long currentMillis = millis();

      previousMillis = currentMillis;
      while (sensorValues[5] != 1000 && sensorValues[4] != 1000 && sensorValues[3] != 1000 ) {
        qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
        right();

        unsigned long powercurrentMillis = millis();

        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

          powerpreviousMillis = powercurrentMillis;
          power = ina260.readPower() + power;
          check_power();

        }
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= interval) {
          lost_tape_check = true;
          previousMillis = currentMillis;
          break;

        }
        test_counter = test_counter + 1;
        if (test_counter >= 200) {
          test_counter = 0;
          analogWrite (enA, 250);
          analogWrite (enB, 250);
          delay(50);

        }
        analogWrite (enA, turnSpeed);
        analogWrite (enB, turnSpeed);

      }
    }
    if (lost_tape_check == true) {
      Serial2.println('A');
      initiation = false;
      stop_motor();
    } else {
      lost_tape_check = false;
      forward();
    }

    test_counter = 0;
    previousMillis = 0;
    lost_tape = 0;
    test = true;
    increase_pid = true;
  }



  //Sväng vänster
  if (c == '3')
  {
    c = 3;
    lost_tape_check = false;

    turn_off_LEDS();
    digitalWrite(L_LED, HIGH);
    qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
    unsigned long currentMillis = millis();

    previousMillis = currentMillis;
    while (sensorValues[0] != 1000) {
      unsigned long powercurrentMillis = millis();

      if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

        powerpreviousMillis = powercurrentMillis;
        power = ina260.readPower() + power;
        check_power();

      }
      qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
      left();
      analogWrite (enA, turnSpeed);
      analogWrite (enB, turnSpeed);
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= interval) {
        lost_tape_check = true;
        previousMillis = 0;
        break;

      }
      test_counter = test_counter + 1;

      if (test_counter >= 200) {
        test_counter = 0;
        analogWrite (enA, 250);
        analogWrite (enB, 250);
        delay(50);
      }

      analogWrite (enA, turnSpeed);
      analogWrite (enB, turnSpeed);
    }
    previousMillis = 0;
    stop_motor();
    delay(100);
    test_counter = 0;
    lost_tape = 0;

    if (lost_tape_check == true) {
      Serial2.println('A');
      initiation = false;
    } else {
      qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
      unsigned long currentMillis = millis();

      previousMillis = currentMillis;
      while (sensorValues[4] != 1000 && sensorValues[3] != 1000) {
        unsigned long currentMillis = millis();
        unsigned long powercurrentMillis = millis();

        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

          powerpreviousMillis = powercurrentMillis;
          power = ina260.readPower() + power;
          check_power();

        }
        if (currentMillis - previousMillis >= 6000) {
          lost_tape_check = true;
          previousMillis = currentMillis;
          break;
        }
        test_counter = test_counter + 1;

        if (test_counter >= 200) {
          test_counter = 0;
          analogWrite (enA, 250);
          analogWrite (enB, 250);
          delay(50);
        }

        qtr.readLineBlack(sensorValues); //Beräknar positionen nav bilen.
        left();
        analogWrite (enA, turnSpeed);
        analogWrite (enB, turnSpeed);
      }

      if (lost_tape_check == true) {
        Serial2.println('A');
        initiation = false;
      } else {
        stop_motor();
        delay(50);
        forward();

      }
    }

    test_counter = 0;
    test = true;
    increase_pid = true;

  }


  if (c == '4') {
    c = 4;
    turn_off_LEDS();
    digitalWrite(B_LED, HIGH);
    reverse();

    analogWrite (enA, 140);
    analogWrite (enB, 140);
    delay(300);

    digitalWrite(MotorAip1, LOW);
    digitalWrite(MotorAip2, LOW);
    digitalWrite(MotorBip1, LOW);
    digitalWrite(MotorBip2, LOW);
    analogWrite (enA, 0);
    analogWrite (enB, 0);
    delay(200);
    position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
    unsigned long currentMillis = millis();

    previousMillis = currentMillis;
    if (last_turn == '3') {
      while (position != 0) {
        position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
        left();

        unsigned long currentMillis = millis();
        unsigned long powercurrentMillis = millis();

        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

          powerpreviousMillis = powercurrentMillis;
          power = ina260.readPower() + power;
          check_power();

        }
        if (currentMillis - previousMillis >= interval) {
          lost_tape_check = true;
          previousMillis = currentMillis;
          break;

        }
        test_counter = test_counter + 1;

        if (test_counter >= 600) {
          test_counter = 0;
          analogWrite (enA, 250);
          analogWrite (enB, 250);
          delay(50);
        }

        analogWrite (enA, turnSpeed);
        analogWrite (enB, turnSpeed);

      }
    }
    else if (last_turn == '2') {
      while (position != 7000) {
        position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
        right();
        unsigned long powercurrentMillis = millis();

        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {

          powerpreviousMillis = powercurrentMillis;
          power = ina260.readPower() + power;
          check_power();

        }
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= interval) {
          lost_tape_check = true;
          previousMillis = currentMillis;
          break;

        }

        test_counter = test_counter + 1;
        if (test_counter >= 600) {
          test_counter = 0;
          analogWrite (enA, 250);
          analogWrite (enB, 250);
          delay(50);
        }

        analogWrite (enA, turnSpeed);
        analogWrite (enB, turnSpeed);

      }
    } else {
      while (position != 7000) {
        position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
        right();
        unsigned long powercurrentMillis = millis();

        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {
          powerpreviousMillis = powercurrentMillis;
          power = ina260.readPower() + power;
          check_power();

        }
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
          lost_tape_check = true;
          previousMillis = currentMillis;
          break;

        }

        test_counter = test_counter + 1;
        if (test_counter >= 600) {
          test_counter = 0;
          analogWrite (enA, 250);
          analogWrite (enB, 250);
          delay(50);
        }

        analogWrite (enA, turnSpeed);
        analogWrite (enB, turnSpeed);

      }
    }

    stop_motor();
    delay(500);
    position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
    forward();

    lost_tape = 0;
    test_counter = 0;
    test = true;
  }



  if (c == '6') {
    c = 6;

    Claw.attach(12);
    delay(50);
    for (int i = Claw.read(); i > 0; i--) {
      Claw.write(i);
      delay(20);

    }
    delay(100);
    Claw.detach();
    Serial2.println('I');


  }

  if (c == '5') {
    c = 5;

    Claw.attach(12);
    delay(50);
    Claw.write(60);

    delay(150);

    Claw.detach();
    delay(500);
    digitalRead(claw_confirm);
    if (digitalRead(claw_confirm)  == LOW) // Som bekräftelse att pucken har lämnats?
    {
      Serial.println("claw low");
      Serial2.println('I');

      turn_off_LEDS();

    }

    if (digitalRead(claw_confirm) == HIGH)
    {
      Serial.println("claw high");
      Serial2.println('B');
      turn_off_LEDS();
      while (digitalRead(claw_confirm) == HIGH) {
        delay(500);
        digitalWrite(Warning_LED, HIGH);
        delay(500);
        digitalWrite(Warning_LED, LOW);
        digitalRead(claw_confirm);
        if (digitalRead(claw_confirm) == LOW)
        {
          Serial.print("Sending: "); Serial.println(c);
          Serial2.println('I');
          digitalWrite(Idle_LED, HIGH);

          break;
        }

      }
      turn_off_LEDS();


    }
  }


  position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.

  if (initiation == true) {
    position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
    if (sensorValues[0] < 100 && sensorValues[1] < 100 && sensorValues[2] < 100 && sensorValues[3] < 100 && sensorValues[4] < 100 && sensorValues[5] < 100 && sensorValues[6] < 100 && sensorValues[7] < 100) {
      int position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.
      stop_motor();
      delay(200);
      while (position == 7000) {
        position = qtr.readLineBlack(sensorValues);
        right();
        analogWrite(enA, 150); //Högermotor hastighet
        analogWrite(enB, 0); //Vänstermotor hastighet

        //Samplar energi varje sekund
        unsigned long powercurrentMillis = millis();
        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {
          powerpreviousMillis = powercurrentMillis;
          check_power();
        }

        //Om bilen fastnar i en sekund, så ska motorn få lite kickstart
        test_counter = test_counter + 1;
        if (test_counter >= 1000) {
          test_counter = 0;
          analogWrite (enA, 250);
          analogWrite (enB, 0);
          delay(50);
        }
        if (position < 6000) {
          break;
        }
        else if (sensorValues[6] == 1000  && sensorValues[5] == 1000  && sensorValues[7] == 1000 ) {
          delay(250);
          stop_motor();
          initiation = false;
          if (test == true) {
            Serial2.println('I');
            test = false;
          }
          Serial.print("Found right crossing");
        }
        else if (sensorValues[0] == 1000  && sensorValues[1] == 1000 && sensorValues[2] == 1000) {
          delay(250);

          stop_motor();
          initiation = false;

          if (test == true) {
            Serial2.println('I');
            test = false;
          }

          Serial.println("Found left crossing");
        }
      }
      previousMillis = 0;


      while (position == 0) {
        position = qtr.readLineBlack(sensorValues); //Beräknar positionen av bilen.

        left();
        analogWrite(enA, 0); //Högermotor hastighet
        analogWrite(enB, 150); //Vänstermotor hastighet

        unsigned long powercurrentMillis = millis();
        if (powercurrentMillis - powerpreviousMillis >= powerinterval) {
          powerpreviousMillis = powercurrentMillis;
          check_power();
        }

        //Om bilen fastnar i en sekund, så ska motorn få lite kickstart
        test_counter = test_counter + 1;
        if (test_counter >= 1000) {
          test_counter = 0;
          analogWrite (enA, 0);
          analogWrite (enB, 250);
          delay(50);
        }
        if (position > 500) {
          break;
        }  else if (sensorValues[6] == 1000  && sensorValues[5] == 1000  && sensorValues[7] == 1000 ) {
          delay(250);
          stop_motor();


          initiation = false;
          if (test == true) {
            Serial2.println('I');
            test = false;

          }
          Serial.print("Found right crossing");

        }
        else if (sensorValues[0] == 1000  && sensorValues[1] == 1000 && sensorValues[2] == 1000) {
          delay(250);

          stop_motor();
          initiation = false;

          if (test == true) {
            Serial2.println('I');
            test = false;
          }

          Serial.println("Found left crossing");
        }
      }

      previousMillis = 0;

      delay(50);
      forward();


    }
  }


  if (rightMotorSpeed >= rightMaxSpeed) {

    rightMotorSpeed = rightMaxSpeed;
  }
  if (leftMotorSpeed >= leftMaxSpeed) {

    leftMotorSpeed = leftMaxSpeed;
  }
  //Rensa det som har skickats.
  c = ' ';

}




void forward() {
  increase_pid = true;

  digitalWrite(MotorAip1, HIGH);
  digitalWrite(MotorAip2, LOW);
  digitalWrite(MotorBip1, HIGH);
  digitalWrite(MotorBip2, LOW);

}


void left() {

  digitalWrite(MotorAip1, LOW);
  digitalWrite(MotorAip2, HIGH);
  digitalWrite(MotorBip1, HIGH);
  digitalWrite(MotorBip2, LOW);

}

void right() {

  digitalWrite(MotorAip1, HIGH);
  digitalWrite(MotorAip2, LOW);
  digitalWrite(MotorBip1, LOW);
  digitalWrite(MotorBip2, HIGH);

}

void reverse() {

  digitalWrite(MotorAip1, LOW);
  digitalWrite(MotorAip2, HIGH);
  digitalWrite(MotorBip1, LOW);
  digitalWrite(MotorBip2, HIGH);
}

void stop_motor() {

  digitalWrite(MotorAip1, LOW);
  digitalWrite(MotorAip2, LOW);
  digitalWrite(MotorBip1, LOW);
  digitalWrite(MotorBip2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);


}
void turn_off_LEDS() {
  digitalWrite(Idle_LED, LOW);
  digitalWrite(R_LED, LOW);
  digitalWrite(F_LED, LOW);
  digitalWrite(L_LED, LOW);
  digitalWrite(B_LED, LOW);
}

void check_power() {
  float power_tower = ina260.readPower();
  if (power_tower >= 0 && power_tower < 2000 ) {
    digitalWrite(POW_R_LED, LOW);
    digitalWrite(POW_Y2_LED, LOW);
    digitalWrite(POW_Y1_LED, LOW);
    digitalWrite(POW_G2_LED, LOW);
    digitalWrite(POW_G1_LED, HIGH);

  }
  if (power_tower >= 2000 && power_tower < 4000) {
    digitalWrite(POW_R_LED, LOW);
    digitalWrite(POW_Y2_LED, LOW);
    digitalWrite(POW_Y1_LED, LOW);
    digitalWrite(POW_G2_LED, HIGH);
    digitalWrite(POW_G1_LED, HIGH);

  }
  if (power_tower >= 4000 && power_tower < 6000) {
    digitalWrite(POW_R_LED, LOW);
    digitalWrite(POW_Y2_LED, LOW);
    digitalWrite(POW_Y1_LED, HIGH);
    digitalWrite(POW_G2_LED, HIGH);
    digitalWrite(POW_G1_LED, HIGH);
  }
  if (power_tower >= 6000 && power_tower < 8000) {
    digitalWrite(POW_R_LED, LOW);
    digitalWrite(POW_Y2_LED, HIGH);
    digitalWrite(POW_Y1_LED, HIGH);
    digitalWrite(POW_G2_LED, HIGH);
    digitalWrite(POW_G1_LED, HIGH);

  }
  if (power_tower >= 8000) {
    digitalWrite(POW_R_LED, HIGH);
    digitalWrite(POW_Y2_LED, HIGH);
    digitalWrite(POW_Y1_LED, HIGH);
    digitalWrite(POW_G2_LED, HIGH);
    digitalWrite(POW_G1_LED, HIGH);

  }
}
