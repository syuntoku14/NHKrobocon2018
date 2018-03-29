void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){;}
  //delay(10000);
}

int poleangle=false;
bool success_flag;
void loop() {
  if (Serial.read()=='c'){
    success_flag=true;
  }
  if(success_flag && Serial.available()>0){
    poleangle=Serial.read();
    //Serial.println(poleangle);
  }
  if(!success_flag){
  Serial.write('r');
  delay(1000);}
}
