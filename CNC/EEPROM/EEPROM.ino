//To move 30cm: number of pulses is 300*9*40 = 108000

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
#define pass_length 9
#define in1 8
#define in2 9
#define enA 10
#define enB 11
#define in3 12
#define in4 13
#define kp 450
#define ki 0.5
#define kd 2.5

byte motor_power, motor_power2;
bool state = 1, state2=1;
double current_error, previous_error = 0, delta_error, current_time, previous_time=0, delta_time, integral_e = 0, P, I, D;
double current_error2, previous_error2 = 0, delta_error2, integral_e2 = 0, P2, I2, D2;
char password[pass_length] = "12345678";
char Data[pass_length];
const byte rows = 4;
const byte columns = 4;
char keys[rows][columns] = {
  {'7','8','9','/'},
  {'4','5','6','x'},
  {'1','2','3','-'},
  {'*','0','#','+'}
};
byte rowPins[rows] = {22,23,24,25};
byte colPins[columns] = {26,27,28,29};
Keypad myKeyPad = Keypad(makeKeymap(keys),rowPins,colPins,rows,columns);
LiquidCrystal_I2C lcd(0x27,20,4);
char key;
int data_count = 0;
byte flag = 0;
bool dcFlag;
String inp = "";
int speedx=0,speedy=0;
byte flag2=0;
double posx=0.0,posy=0.0;
double posX,posY;
int pulseX=0,pulseY=0;
int speed_count = 0, pos_count=0;

void X_ISR(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
}

void Y_ISR(){
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}

void encoderX(){
  if(digitalRead(6)) pulseX++;
  else pulseX--;
  posx = pulseX/360.0;
}

void encoderY(){
  if(digitalRead(5)) pulseY++;
  else pulseY--;
  posy = pulseY/360.0;
}

void setup() {
  // put your setup code here, to run once:
  lcd.begin(20,4);
  lcd.setCursor(0,0);
  lcd.print("Enter Password");
  for (int i = 0; i < pass_length-1; i++) {
    password[i] = EEPROM.read(i);}
  Serial.begin(9600);
  Serial.println(password);
  pinMode(32,INPUT_PULLUP);
  pinMode(33,INPUT_PULLUP);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3),X_ISR,FALLING);
  attachInterrupt(digitalPinToInterrupt(2),Y_ISR,FALLING);
  attachInterrupt(digitalPinToInterrupt(18),encoderY,RISING);
  attachInterrupt(digitalPinToInterrupt(19),encoderX,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(flag==0){
  lcd.setCursor(0,0);
  lcd.print("Enter Password");
  key = myKeyPad.getKey();
  if(key){
    Data[data_count] = key;
    lcd.setCursor(data_count,1);
    lcd.print(key);
    data_count++;   
  }
  if(data_count == pass_length-1){
    lcd.clear();
    data_count = 0;
    if(!strcmp(Data,password)){
      lcd.print("Correct");
      delay(1000);
      flag = 1;
      lcd.clear();
    }
    else {lcd.print("Incorrect");delay(500);lcd.clear();}
  }
 }
  if(flag==1){
    lcd.setCursor(0,0);
    lcd.print("Enter New One");
    lcd.setCursor(0,1);
    lcd.print("8 digits");
    key = myKeyPad.getKey();
  if(key){
    lcd.setCursor(data_count,2);
    lcd.print(key);
    EEPROM.write(data_count,key);
    data_count++;   
  }
  if(data_count == pass_length-1) {flag = 2;lcd.clear();}
  }
  if(flag==2){
    lcd.setCursor(0,0);
    lcd.print("1 for manual");
    lcd.setCursor(0,1);
    lcd.print("2 for automatic");
    key = myKeyPad.getKey();
    if(key){
      if(key=='1') {flag=3;lcd.clear();}
      if(key=='2') {flag=4;lcd.clear();}
    }
  }
  if(flag==3){
    if(flag2==0){
      lcd.setCursor(0,0);
      lcd.print("Choose axis");
      lcd.setCursor(0,1);
      lcd.print("1 for x");
      lcd.setCursor(0,2);
      lcd.print("2 for y");
      lcd.setCursor(0,3);
      lcd.print("pos(mm)= ");
      lcd.print(posx);
      lcd.print(",");
      lcd.print(posy);
      key = myKeyPad.getKey();
      if(key){
        if(key=='1') {dcFlag=1;}
        if(key=='2') {dcFlag=0;}
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Choose speed");
        lcd.setCursor(0,1);
        lcd.print("press = to enter");
        flag2=1;
    }}
    if(flag2==1){
    key = myKeyPad.getKey();
    if(key){
    if(key == '#'){
      if(dcFlag) speedx = inp.toInt();
      else speedy = inp.toInt();
      inp = "";
      lcd.clear();
      speed_count = 0;
      flag2=0;
    }
    else{
      inp += key;
      lcd.setCursor(speed_count,2);
      lcd.print(key);
      speed_count++;
    }
  }}
  if(speedx>255) speedx=255;
  if(speedy>255) speedy=255;
  
  if(!digitalRead(32)){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA,speedx);
    lcd.clear();
    flag2=3;
  }
  if(flag2==3) {lcd.setCursor(0,0);lcd.print("Moving on x");
    lcd.setCursor(0,1);lcd.print("with speed ");lcd.print(speedx);
    lcd.setCursor(0,2);lcd.print(posx);}
    
  if(!digitalRead(33)){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB,speedy);
    lcd.clear();
    flag2=4;
  }
  if(flag2==4) {lcd.setCursor(0,0);lcd.print("Moving on y");
    lcd.setCursor(0,1);lcd.print("with speed ");lcd.print(speedy);
    lcd.setCursor(0,2);lcd.print(posy);}

  if(!digitalRead(3)){
    flag2 = 0;
    lcd.clear();
  }
  if(!digitalRead(2)){
    flag2 = 0;
    lcd.clear();
  }
  }
  if(flag==4){
    if(flag2==0){
      lcd.setCursor(0,0);
      lcd.print("enter posX:");}
    else if(flag2==1){
      lcd.setCursor(0,0);
      lcd.print("enter posY:");}
   else{
    current_error = posX - posx;
    delta_error = current_error - previous_error;
    previous_error = current_error;

    current_error2 = posY - posy;
    delta_error2 = current_error2 - previous_error2;
    previous_error2 = current_error2;

    // time
    current_time = micros();
    delta_time = (current_time - previous_time) / 1e6;
    previous_time = current_time;
  
    // Params;
    P = kp * current_error;
    D = kd * delta_error/delta_time;
    P2 = kp * current_error2;
    D2 = kd * delta_error2/delta_time;
  
    integral_e += current_error * delta_time;
    I = ki * integral_e;
    integral_e2 += current_error2 * delta_time;
    I2 = ki * integral_e2;
    
    double Signal = P + I + D;
    double Signal2 = P2 + I2 + D2;
  
    motor_power = fabs(Signal);
    motor_power2 = fabs(Signal2);
  
    if( Signal > 0) state = 1;
    else state = 0;

    if( Signal2 > 0) state2 = 1;
    else state2 = 0;

    if(state){
      digitalWrite(in2, HIGH);
      digitalWrite(in1, LOW);
    }
    else{
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    if(state2){
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
    else{
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }
    analogWrite(enA, motor_power);
    analogWrite(enB, motor_power2);
    
    lcd.setCursor(0,0);
    lcd.print("posx (mm): ");
    lcd.print(posx);
    lcd.setCursor(0,1);
    lcd.print("posy (mm): ");
    lcd.print(posy);
  }
    
    key = myKeyPad.getKey();
    if(key){
    if(key == '#'){
      if(flag2==0) posX = inp.toInt();
      else posY = inp.toInt();
      flag2++;
      inp = "";
      lcd.clear();
      pos_count = 0;
    }
    else{
      inp += key;
      lcd.setCursor(pos_count,1);
      lcd.print(key);
      pos_count++;}
  }
  }
  




}
