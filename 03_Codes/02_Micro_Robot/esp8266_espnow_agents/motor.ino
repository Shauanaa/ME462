void motorControl(int sol,int sag){
  if(sol < 0){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }else{
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }
  analogWrite(ENA,abs(sol));
  if(sag < 0){
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  }else{
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  }
  analogWrite(ENB,abs(sag));
}