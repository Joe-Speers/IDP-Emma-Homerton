#define hallsensorPin A0
int hallreading;
void setup(){
  Serial.begin(9600);
  pinMode(4,OUTPUT);//control LED lignt turn on and off
}
void loop(){
  hallreading=analogRead(hallsensorPin);
  delay(500);//to be discussed the interval between 2 detecting
  if(hallreading<510){//can adjust accuracy
    Serial.println("found it");
    digitalWrite(4,HIGH);
    delay(500);
    digitalWrite(4,LOW);
  }
  else{
    Serial.println("no found");
  }
}