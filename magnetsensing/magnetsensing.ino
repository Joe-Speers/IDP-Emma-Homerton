#define hallsensorPin A0
int hallreading;
void setup(){
  Serial.begin(9600);
  pinMode(4,OUTPUT);
}
void loop(){
  hallreading=analogRead(hallsensorPin);
  delay(500);
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