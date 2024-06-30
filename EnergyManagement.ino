#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

int inputPin = 4;  
int LDR = A0;
int Fan = 5;
int Bulb = 6;
int Bulb_2 = 8;
int pirState = LOW; 

int val = 0;  

int tempLimit;
int tempLimitAddress = 0;

const int sensorPin = A1;
float sensorValue;
float voltageOut;

float temperatureC;
float temperatureF;
float temperatureK;

const int currentSensorPin = A2; 
const float sensorOffset = 2.5; 
const float sensitivity = 185;   

const int numReadings = 100;

const int buttonPin1 = 2;
const int buttonPin2 = 3;

volatile boolean buttonState1 = HIGH;       
volatile boolean lastButtonState1 = HIGH; 
volatile unsigned long lastDebounceTime1 = 0; 
volatile unsigned long debounceDelay1 = 50;    

volatile boolean buttonState2 = HIGH;
volatile boolean lastButtonState2 = HIGH;
volatile unsigned long lastDebounceTime2 = 0;
volatile unsigned long debounceDelay2 = 50;

enum State{
  IDLE,
  FAN_ON,
  BULB_ON,
  FAN_BULB_ON,
  FAN_BULB_OFF
};

State currentState = IDLE;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);

  pinMode(sensorPin, INPUT);
  pinMode(Fan, OUTPUT);
  pinMode(Bulb, OUTPUT);
  pinMode(Bulb_2, OUTPUT);
  pinMode(inputPin, INPUT_PULLUP); 
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  digitalWrite(Fan, LOW);
  digitalWrite(Bulb, LOW);
  digitalWrite(Bulb_2, LOW);
  attachInterrupt(digitalPinToInterrupt(buttonPin1), buttonInterrupt1, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), buttonInterrupt2, FALLING);

  tempLimit = EEPROM.read(tempLimitAddress);

  // if (tempLimit < 10 || tempLimit > 100) {
  //   tempLimit = 40;
  // }

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("SYSTEM");
  lcd.setCursor(1, 1);
  lcd.print("INITIALIZATION");
  delay(2000);

}

void loop() {
  checkButtonPresses();
  switch (currentState) {
    case IDLE:
      idle();
      break;
    case FAN_ON:
      fan_on();
      break;
    case BULB_ON:
      bulb_on();
      break;
    case FAN_BULB_ON:
      fan_bulb_on();
      break;
    case FAN_BULB_OFF:
      fan_bulb_off();
      break;
  }
  int LightValue = analogRead(LDR);
  //Serial.println(LightValue);
  sensorValue = analogRead(sensorPin);
  voltageOut = (sensorValue * 5000) / 1024;

  temperatureK = voltageOut / 10;
  temperatureC = temperatureK - 273;
  temperatureF = (temperatureC * 1.8) + 32;
  Serial.println(temperatureC);

  val = digitalRead(inputPin);  // read input value
  delay(500);
  if (val == HIGH) {             // check if the input is HIGH
    if (pirState == LOW) {
      digitalWrite(Bulb, HIGH);
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    if (pirState == HIGH) {
      digitalWrite(Bulb, LOW); 
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }

  if (LightValue < 300) {
    digitalWrite(Bulb_2, HIGH);
  }else{
    digitalWrite(Bulb_2, LOW);
  }

  if (temperatureC > tempLimit) {
    digitalWrite(Fan, HIGH);
  } else {
    digitalWrite(Fan, LOW);
  }

}
void idle(){
  if(digitalRead(Fan) == LOW && digitalRead(Bulb) == LOW){
    currentState = FAN_BULB_OFF;
  }else if (digitalRead(Fan) == HIGH && digitalRead(Bulb) == LOW){
    currentState = FAN_ON;
  }else if (digitalRead(Fan) == LOW && (digitalRead(Bulb) == HIGH || digitalRead(Bulb_2) == HIGH) ){
    currentState = BULB_ON;
  }else if (digitalRead(Fan) == HIGH && (digitalRead(Bulb) == HIGH || digitalRead(Bulb_2) == HIGH)){
    currentState = FAN_BULB_ON;
  }
}
void fan_bulb_off(){
  idle();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FAN: OFF ");
  lcd.print("  L: ");
  lcd.print(tempLimit);
  lcd.setCursor(0,1);
  lcd.print("BULB: OFF");
  delay(1000);
}
void fan_on(){
  idle();
  float totalVoltage = 0;
  for (int i = 0; i < numReadings; i++) {
    int sensorValue = analogRead(currentSensorPin); 
    float voltage = sensorValue * (5.0 / 1023.0);  
    totalVoltage += voltage;
    delay(10);
  }
  
  float avgVoltage = totalVoltage / numReadings;
  // Serial.print("Avg Voltage: ");
  // Serial.print(avgVoltage);
  // Serial.print(" V");

  float current = (avgVoltage - sensorOffset) / (sensitivity / 1000); 

  if(current < 0){
    current = current * -1;
  }
  float Power = 12 * current;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FAN: ON ");
  lcd.print("T: ");
  lcd.print(temperatureC);
  lcd.setCursor(0,1);
  lcd.print("POWER: ");
  lcd.print(Power);
  lcd.print(" W");
  delay(1000);
}
void bulb_on(){
  idle();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("BULB: ON");
  lcd.print("  L: ");
  lcd.print(tempLimit);
  lcd.setCursor(0,1);
  lcd.print("POWER: ");
  lcd.print("65W");
  delay(1000);
}
void fan_bulb_on(){
  idle();
  float totalVoltage = 0;
  for (int i = 0; i < numReadings; i++) {
    int sensorValue = analogRead(currentSensorPin); 
    float voltage = sensorValue * (5.0 / 1023.0); 
    totalVoltage += voltage;
    delay(10); 
  }
  
  float avgVoltage = totalVoltage / numReadings; 
  Serial.print("Avg Voltage: ");
  Serial.print(avgVoltage);
  Serial.print(" V");

  float current = (avgVoltage - sensorOffset) / (sensitivity / 1000); 

  if(current < 0){
    current = current * -1;
  }
  float Power = 12 * current;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FAN: ON ");
  lcd.print("T: ");
  lcd.print(temperatureC);
  lcd.setCursor(0,1);
  lcd.print("BULB: ON");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("F POWER: ");
  lcd.print(Power);
  lcd.print(" W");
  lcd.setCursor(0,1);
  lcd.print("B POWER: ");
  lcd.print("65W");
  delay(1000);
}
void buttonInterrupt1() {
  if (millis() - lastDebounceTime1 > debounceDelay1) {
    buttonState1 = digitalRead(buttonPin1);
    if (buttonState1 == LOW && lastButtonState1 == HIGH) {
      Serial.println("Button 1 Pressed");
      tempLimit--;
      if (tempLimit < 10) tempLimit = 10;

      if (EEPROM.read(tempLimitAddress) != tempLimit) {
        EEPROM.write(tempLimitAddress, tempLimit);
      }
    }
    lastButtonState1 = buttonState1;
    lastDebounceTime1 = millis();
  }
}

void buttonInterrupt2() {
  if (millis() - lastDebounceTime2 > debounceDelay2) {
    buttonState2 = digitalRead(buttonPin2);
    if (buttonState2 == LOW && lastButtonState2 == HIGH) {
      Serial.println("Button 2 Pressed");
      tempLimit++;
      if (tempLimit > 100) tempLimit = 100;
      if (EEPROM.read(tempLimitAddress) != tempLimit) {
        EEPROM.write(tempLimitAddress, tempLimit);
      }
    }
    lastButtonState2 = buttonState2;
    lastDebounceTime2 = millis();
  }
}
void checkButtonPresses() {
  buttonInterrupt1();
  buttonInterrupt2();
}
