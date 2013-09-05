
double angle=0;
double error=0;
void setup()
{
  int temp=0;
  Serial.begin(115200);
  Serial.print("Adjusting");
  temp=analogRead(A0) *0.35-180;
  while((temp+error)!=0)
  {
    if(temp!=0)
    {
      if(temp>0)
      {error-=1;}
      else
      {error+=1;}
    }
    Serial.print(".");
  }
  Serial.print("End\n");
  Serial.println("Measure Start");
}
void loop()
{
    angle = analogRead(A0) *0.35-180+error ;
    if(angle>180)
    {angle-=360;}
    Serial.println(angle);
    delay(100);
  
}
