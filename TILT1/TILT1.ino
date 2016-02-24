const int statusLED = 13;
const int switchTilt = 2;
int val = 0;
void setup(){
   pinMode (statusLED,OUTPUT);
   pinMode (switchTilt,INPUT);
}
void loop(){
   val = digitalRead(switchTilt);
   if (val == HIGH){
      digitalWrite(statusLED,HIGH);
   }
   else {
   digitalWrite(statusLED,LOW);
   }
}
