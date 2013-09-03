#include <avr/io.h>  
#include <avr/interrupt.h>  
#include <D_stepper.h>
#include <math.h>
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
                                     // for your motor
// initialize the stepper library on pins 8 through 11:
D_stepper myStepper(stepsPerRevolution, 12,11,10,9 , //12,11,10,9
                    stepsPerRevolution ,7,6,5,4);    //7,6,5,4        
int stepA_number , stepB_number , number_of_steps = 200;  // work for stepper
float wheelDiameter , tread; //set these to calculate the distance
int pattern = 0;
int xx ; // sever for case 71 ,72 ,77,78  48 steps for 5cm 149 steps for 15cm
int yy;  // for pattern
//  ultrasonic
const int pingPinL = 50;
const int pingPinL2 = 51;
const int pingPinR = 2;
const int pingPinR2 = 3;
long  cmL  ,cmR , last_cmL , last_cmR;
///  all kinds of sensor 
unsigned char FireSensor;
unsigned char LineSensor;
int flag10 = 1;
//  
//flag1 for for Y direction transit
// time1_flag  for time count flag
// flag2 for ultrasonic  //open or close in block 2
// 
int LineCountX = 0; // initial position  ,,,about 21cm from the border
int LineCountY = 0;  //initial position 
int step_count_x = 0;     // low level of distance
int step_count_y = 0;
int stepCount = 0;         // number of steps the motor has taken
int gl_angle = -90;            // total car angle   ,,,initial angle 
int block ;    //for point out which block we are
  int Fire_flag = 0; // what kind of  fire it is 
int fire = 0;  // how many fire have been put out
int FireDirection = 0;  //0 for no direction ; 1 for left ;2 for right
int flag1 = 0;
int flag2 = 0;
int temp=0;
int CandleCount = 0;

//
int time1_flag = 1;
/* 
 * 将定时器中断设为1ms 
 */  
void init_time()  
{  
  
    TCCR1B |= ( 1 << CS11 ) | ( 1 << CS10 ); //64
    TCNT1 = 0x06ff; 
    TIMSK1 |= ( 1 << TOIE1 ); 
   // sei();  //the same as the I bit in the SREG register
}  
long last_time_L;

void pingL(){
  long duration ;
  if( millis() - last_time_L > 65.0){ 
    last_time_L = millis();

    digitalWrite(pingPinL, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPinL, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPinL, LOW);
  

    duration = pulseIn(pingPinL2, HIGH);
      cmL = duration/ 29 / 2;
      if ( CandleCount >= 3) 
          cmL = 300;
      Serial.println(cmL);
  }
}


///  for fan in pin D40 D41 D39
void fan(int f){
  switch( f ){
    case 1 :
      digitalWrite(A10 ,HIGH); //left      
      delay(5500);
      digitalWrite(A10 , LOW);
      break;
   case 2 :    //right
      digitalWrite(A9 ,HIGH);
      delay(5500);
      digitalWrite(A9 , LOW);
      break;
   case 3 : //head
      digitalWrite(A8 ,HIGH);
      delay(5500);
      digitalWrite(A8 , LOW);
      break;
  }  
  CandleCount++;
}


void buz(int k){
  int i , j;
  switch(k){ 
   case 1:  // start ,,,to go
    for (i = 0 ; i < 500 ; i++){
        digitalWrite(48 , HIGH);
        delayMicroseconds(100); 
        digitalWrite(48 , LOW);
        delayMicroseconds(100); 
    }
    break;
   case 2: //have detected the fire
     for( j = 0 ; j < 3 ; j++){
         for (i = 0 ; i < 900 ; i++){
            digitalWrite(48 , HIGH);
            delayMicroseconds(100); 
            digitalWrite(48 , LOW);
            delayMicroseconds(100);
        }   
            delay(800);
     }
     break;
   case 3: // have put out the fire
     for( j = 0 ;j < 2 ; j++){
            for (i = 0 ; i < 900 ; i++){
            digitalWrite(48 , HIGH);
            delayMicroseconds(100); 
            digitalWrite(48 , LOW);
            delayMicroseconds(100);
        } 
        delay(300);  
     }
     break;
    case 4: // in home
      for( j = 0 ;j < 4 ; j++){
            for (i = 0 ; i < 900 ; i++){
            digitalWrite(48 , HIGH);
            delayMicroseconds(100); 
            digitalWrite(48 , LOW);
            delayMicroseconds(100);
        } 
        delay(300);  
     }
  }
}


// our sensor connect to 
/*       |1|
     2-   -3 */
     // PC0 37   PC7 y30
     
     //  4  3(the best)  2 1 0

int CountLine(){
       unsigned char b;
       b = PINA;
  //     if( ( b != 0 )&& ( stepCount >= 268 ) )  // 268 steps about 27cm  long 
       if  ( ( b == 3) && ( stepCount >= 29 ) ){  // 39 steps about 4 cm
           return 1;
       }
       else if( stepCount >= 298 ){ // 298 steps about 30cm  // to avoid leak the crossLine           
           return 1;
       }
       else
          return 0;
 //   line = PINX & 0000 0xxx
   // lineSensor 
 //  return (line && 0xff);
     // and make sure stepCount >=  278 steps  about 28cm
} 
int countA = 0; 
SIGNAL(SIG_OVERFLOW1) 
{  
    TCNT1 = 0x06ff; 
    if ( time1_flag ){    
      ++countA;
      if( countA == 4 ) //5ms 
      {  

         stepCount++;
         stepA_number++;
         if ( stepA_number == number_of_steps )
            stepA_number = 0;
          countA=0; 
          switch (stepA_number%4) {
            
          case 0:    // 1010
          PORTH = _BV(PH4);
          PORTH = ~(1 << PH3); 
          PORTE = _BV(PE3);
          PORTG = ~(1 << PG5);
          break;
          case 1:    // 0110
          PORTH = ~(1 << PH4);
          PORTH = _BV(PH3);  
          PORTE = _BV(PE3);
          PORTG = ~(1 << PG5);          
          break;
          case 2:    //0101
          PORTH = ~(1 << PH4);
          PORTH = _BV(PH3); 
          PORTE = ~(1 << PE3);
          PORTG = _BV(PG5);
          break;
          case 3:    //1001
          PORTH = _BV(PH4);
          PORTH = ~(1 << PH3); 
          PORTE = ~(1 << PE3);
          PORTG = _BV(PG5);         
          break;
          /*
          case 0:    // 1010
          PORTC = _BV(PC3);
          PORTC = ~(1 << PC2); 
          PORTC = _BV(PC1);
          PORTC = ~(1 << PC0);
          break;
          case 1:    // 0110
          PORTC = ~(1 << PC3);
          PORTC = _BV(PC2);  
          PORTC = _BV(PC1);
          PORTC = ~(1 << PC0);          
          break;
          case 2:    //0101
          PORTC = ~(1 << PC3);
          PORTC = _BV(PC2); 
          PORTC = ~(1 << PC1);
          PORTC = _BV(PC0);
          break;
          case 3:    //1001
          PORTC = _BV(PC3);
          PORTC = ~(1 << PC2); 
          PORTC = ~(1 << PC1);
          PORTC = _BV(PC0);         
          break;
          */
        }
         stepB_number++;
         if ( stepB_number == number_of_steps )
             stepB_number = 0;
         switch (stepB_number%4) {
          
          case 0:    // 1010
         digitalWrite(12, HIGH);
          digitalWrite(11, LOW);
          digitalWrite(10, HIGH);
          digitalWrite(9, LOW);
          break;
          case 1:    // 0110
          digitalWrite(12, LOW);
          digitalWrite(11, HIGH);
          digitalWrite(10, HIGH);
          digitalWrite(9, LOW);
          break;
          case 2:    //0101
          digitalWrite(12, LOW);
          digitalWrite(11, HIGH);
          digitalWrite(10, LOW);
          digitalWrite(9, HIGH);
          break;
          case 3:    //1001
          digitalWrite(12, HIGH);
          digitalWrite(11, LOW);
          digitalWrite(10, LOW);
          digitalWrite(9, HIGH);
          break;
          /*
          case 0:    // 1010
          PORTC = _BV(PC4);
          PORTC = ~(1 << PC5); 
          PORTC = _BV(PC6);
          PORTC = ~(1 << PC7);
          break;
          case 1:    // 0110
          PORTC = ~(1 << PC4);
          PORTC = _BV(PC5);  
          PORTC = _BV(PC6);
          PORTC = ~(1 << PC7);          
          break;
          case 2:    //0101
          PORTC = ~(1 << PC4);
          PORTC = _BV(PC5); 
          PORTC = ~(1 << PC6);
          PORTC = _BV(PC7);
          break;
          case 3:    //1001
          PORTC = _BV(PC4);
          PORTC = ~(1 << PC5); 
          PORTC = ~(1 << PC6);
          PORTC = _BV(PC7);         
          break;
          */
        }
      
      
    }
  }  
} 
/* 
*/  
void setup(void)  
{  
 //   Serial.begin(115200);    
  Serial.begin(9600);  
  init_time(); 
  myStepper.setWheelDiameter(64.0); //65mm
  myStepper.setTread(170.0); 
  char ch;
  pinMode(48,OUTPUT); //rr
  pinMode(13,OUTPUT);
 pinMode(A8,OUTPUT);//fan
//  DDRG = _BV(PG2);
  pinMode(A9,OUTPUT);//fan
  pinMode(A10,OUTPUT);//fan
//  digitalWrite(39,LOW);//fan
  digitalWrite(40,LOW);//fan
  digitalWrite(41,LOW);//fan
  pinMode(pingPinL, OUTPUT);
  pinMode(pingPinL2, INPUT);
  pinMode(pingPinR, OUTPUT);
  pinMode(pingPinR2, INPUT);
  //pinMode(22 , INPUT);
  DDRA = 0x00; // port A for crossline sensor
  PORTA = 0x00;
    
}  


void loop( void )  
{ buz(1); 
  myStepper.setSpeed(50); //30 revolutions per minutes
  char ch;
  pingL();

  last_cmL = cmL;

  
  delay(1000);
  sei(); 
 
  while(1){
  
     /*     if(Serial.available()){
            ch = Serial.read();
          }
            if ( ch == 'a'){
            time1_flag = 0;
            delay(500);
            myStepper.angle(1080);
            delay(500);
  
            }
            if (ch  == 's' )
              time1_flag = 1;  */
              /*
                case 0:
                case 1:
              
              */
/*
   case 2X :  for turn turn
   case 3x :  for hang out and search for fire  ; it is 7 block here
   case 4x :   for put out the fire
   caes 5x     for go home 
*/

              switch( pattern ){
                  case 0 :  
                    pattern = 31;                
                    break;
                  case 1 :
                    if( CountLine()){ // test
                    //  buz(1);
                    time1_flag = 0;
                    //digitalWrite(13 , HIGH);
                     stepCount = 0;
                     delay(800);
                 }
                 else 
                 {
                   time1_flag = 1;
                 //   digitalWrite(13 , LOW) ;
                 }
                    break;
                  case 2 :
                    pattern =2;
                    break;
                  case 11:  // test
                 time1_flag = 0; 
                    myStepper.angle(95);
                    delay(1000);
                    break;
                    //////////   turn the turn
                   case 21:
                     time1_flag = 0;
                     delay(300);
                     myStepper.angle( 95 );
                     delay(300);
                     pattern = 32;
                     gl_angle = 0;
                     stepCount = 0;
                     time1_flag = 1;
                     break;
                   case 22:
                     time1_flag = 0 ;
                     delay(500);
                     myStepper.angle(95);
                     gl_angle = 90;
                     time1_flag = 1;
                     pattern = 33;
                     break;
                   case 23 :
                     time1_flag = 0 ;
                     delay(300);
                     myStepper.angle(95);
                     gl_angle = 180;
                     delay(300);
                     time1_flag = 1;
                     pattern = 34;
                     stepCount=0;
                     break;                   
                     ///////  hang out and search  
                   case 31:  
                     block = 1;        
                     if( CountLine() ){
                        LineCountY++;
                        stepCount = 0;
                        step_count_y = 0;
                        Serial.println("crossLine");
                     }
                     if( LineCountY == 3 ){
                         if( stepCount >= 149 ){  // 149 steps about 15cm
                           pattern = 21 ; //   it is time to turn turn  
                         step_count_y = stepCount;                     
                         }
                        }
                     break;
                   case 32:
                     block = 2;                 
                      // this part is ultrasonic
                       pingL();
 
  //                     if( abs(last_cmL - cmL) >100 ){
  //                         flag2 =~ flag2;
  //                     }
                       if( flag2 ){    // in order to control the left side of ultrasonic                          
                           if (cmL>=6.0 && cmL < 11.0 ) {
                               FireDirection = 1;  // determine which direction to turn when put out fire
                                 pattern = 43;
                                 step_count_x = stepCount;
                             }
                             if(cmL < 6.0){
                                FireDirection = 1; 
                                pattern = 72;
                                xx = 20;
                                yy  = 43;
                                step_count_x = stepCount;
                             }
                             if ( cmL >= 11.0 && cmL < 20.0) {
                               FireDirection = 1;  // determine which direction to turn when put out fire
                            //     pattern = 44;
                               pattern = 72; // X  direction
                              xx = 149 ;
                             yy = 44; 
                               step_count_x = stepCount;
                             }
                             if ( cmL >= 20.0 && cmL <= 37.0) {
                               FireDirection = 1;  // determine which direction to turn when put out fire
                            //     pattern = 44;
                               pattern = 72; // X  direction
                              xx = 178 ;
                             yy = 44; 
                               step_count_x = stepCount;
                             }
                       }

                     
                     if( CountLine() ){
                        LineCountX++;
                        stepCount = 0;
                      //  step_count_x = 0;
                     }
               //      if( LineCountX == 2 && ( abs(cmL - last_cmL) > 100 )){
                     if( LineCountX == 2 && ( stepCount > 84) ){
                         flag2 = 1;
                     }
                     last_cmL = cmL; 
                     if( LineCountX == 5 ){
                         if( stepCount > 149 ){
                           pattern = 22 ; //   it is time to turn turn  
                           step_count_x = 149;             // step_count_x = stepCount;
                         }
                        }                     
                     break;
                   case 33:
                   if (flag10){
                     block = 3;
                     
                     pingL();

                       // this part is ultrasonic
                         if ( cmL >=6.0 && cmL <= 11.0 ) {
                           FireDirection = 1;  // determine which direction to turn when put out fire
                             pattern = 43;
                             step_count_y = stepCount;
                         }   
                         if ( cmL < 6.0) {
                             FireDirection = 1;  // determine which direction to turn when put out fire
                             pattern = 77;
                             xx= 19;
                             yy =43;
                             step_count_y = stepCount;
                         }  
                         if ( cmL >= 11.0 && cmL < 20.0) {
                               FireDirection = 1;  // determine which direction to turn when put out fire
                            //     pattern = 44;
                               pattern = 77; // Y  direction
                              xx = 149 ;
                             yy = 44; 
                               step_count_x = stepCount;
                             }
                         if ( cmL >= 20.0 && cmL <= 37.0) {
                               FireDirection = 1;  // determine which direction to turn when put out fire
                            //     pattern = 44;
                               pattern = 77; // Y  direction
                              xx = 178 ;
                             yy = 44; 
                               step_count_x = stepCount;
                             }

              }   
               //      if( stepCount >= step_count_y ){
                  if( CountLine() ){
                        LineCountY--;
                        stepCount = 0;
                        step_count_y = 0;
                     }
                     if( LineCountY == 0 ){
                         if( stepCount >= 149 ) { // 149 steps about 15cm
                           pattern = 23 ; //   it is time to turn turn  
                           step_count_y = stepCount;
                         }
                     }
                     break;                                   
                   case 34:
                     block = 4;
                     
                     pingL();

                    // this part is ultrasonic
                         if ( cmL >= 6.0 && cmL < 11.0 ) {
                           FireDirection = 1;  // determine which direction to turn when put out fire
                             pattern = 43;
                             step_count_x = stepCount;
                         }
                         if ( cmL < 6.0 ) {
                           FireDirection = 1;  // determine which direction to turn when put out fire
                             pattern = 78;
                             xx= 17;
                             yy = 43;
                             step_count_x = stepCount;
                         }
                         if ( cmL >= 11.0 && cmL < 20.0) {
                           FireDirection = 1;  // determine which direction to turn when put out fire
                        //     pattern = 44;
                           pattern = 78; // X  direction 
                           xx= 149;
                           yy = 44;
                           step_count_x = stepCount;
                         }
                         if ( cmL >= 20.0 && cmL <= 37.0) {
                           FireDirection = 1;  // determine which direction to turn when put out fire
                        //     pattern = 44;
                           pattern = 78; // X  direction 
                           xx= 178;
                           yy = 44;
                           step_count_x = stepCount;
                         }

                     
                     
                         if( CountLine() ){
                            LineCountX--;
                            stepCount = 0;
                            step_count_x = 0;
                         }
                         if( LineCountX == 2 ){                             
                               pattern = 66 ; //   it is time to turn turn                                 
                               stepCount = 0;
                               step_count_x = stepCount;
                               time1_flag = 0 ;                             
                         }                        
                     break; 
                              
                     ///////  put out the fire
                   case 41:  // fire is in the head of the car
                     time1_flag = 0;
                     buz(2);
                     fan(3);   // open the head of the fan
                     delay(200);
                     buz(3);
                     pattern = 45;
                     break;
                     case 42: // near the head of the car
                       time1_flag = 0;                      
                       switch( FireDirection ){
                        case 1:
                          myStepper.angle( 40 );// turn left
                          delay(200);
                          buz(2);
                         fan(3); // open the head of the fan
                         delay(200);
                         buz(3);
                         myStepper.angle( -40 );
                         break;
                       case 2:
                          myStepper.angle( -40 );// turn left
                          delay(200);
                          buz(2);
                         fan(3); // open the head of the fan
                         delay(200);
                         buz(3);
                         myStepper.angle( 40 );
                         break;                         
                       }
                       pattern = 45;
                     break;
                     case 43: // open the side fan
                        time1_flag = 0;  // stop the car
                        switch ( FireDirection ){
                          case 1 :
                            delay(100);
                            buz(2);
                            fan(1); //open the left of the fan
                            buz(3);
                           break;
                          case 2:
                           delay(100);
                           buz(2);
                             fan(2); //open the right of the fan
                            buz(3);
                           break;
                        }
                        pattern = 45;
                     break;
                     case 44: // the fire is far from the car
                        time1_flag = 0; // stop the car
                        switch ( FireDirection ){
                          case 1 :
                            delay(500);
                            myStepper.angle( 95 );// turn left//////
                            gl_angle += 90;  // global angle of the car
                           //
                            temp =(int)(( cmL - 16.0)*10);//
                            
                            if(temp > 0){//
                              myStepper.step(temp,temp);}//
                            buz(2);
                             fan(3); //open the left of the fan
                            delay(200);
                            buz(3);
                            if(temp > 0){//
                             myStepper.step(-temp,-temp);}//
                            delay(200);
                            // go home
                           break;
                          case 2:
                            delay(500);
                            myStepper.angle( -95 );
                            gl_angle -=90;
                            buz(2);
                            fan(3); // open the head   fan
                            delay(200);
                            buz(3);
                            delay(200);
                            // go home
                           break;
                           
                        }
                        pattern = 45;
                   case 45 :  // consider the value block to decide which way to go home
                       stepCount = 0;
                      switch (block){
                         case 1:  // if the car is in block 1 ,,,go to case 51;
                            pattern = 51;
                           break;
                         case 2:
                          pattern = 52;
                          break;
                         case 3:  //block 3
                          pattern = 53;
                          break; 
                         case 4 :
                           pattern = 54 ;                          
                           break;                         
                      }
                      break; 
     ////////////////  begin to go home   ------turn turn first
                   case 51:
                     if( gl_angle == -180){
                         myStepper.angle( -95 );
                        gl_angle = 90; 
                     }
                     if( gl_angle == 0 ){
                         myStepper.angle( 95 );////////
                        gl_angle = 90; 
                     }
                     if( gl_angle == -90){
                        switch(FireDirection){
                         case  1 :
                           myStepper.angle( -188 );  //  inorde not to crash the fire
                           gl_angle = 90; 
                           break;
                         case 2 :
                           myStepper.angle( 188 );
                           gl_angle = 90;
                          break; 
                         default :
                           myStepper.angle( 188 );
                           gl_angle = 90;
                           break;
                       }
                     }
                     pattern = 61 ;  
                     stepCount = 0;
                     break; 
                   case 52 :
                     time1_flag = 0;
                     switch(gl_angle){
                      case 90 :
                       delay(300);
                       myStepper.angle(95);
                       delay(300);
                       break;
                     case 0:
                     delay(300);   // the angle of it can't be -90 ,
                     myStepper.angle(188 );  //  
                     gl_angle = 180;
                     delay(300);
                     break;
                     }
                     pattern = 62;
                     stepCount = 0;
                     break;
                   case 53 :
                     time1_flag = 0;
                     switch( gl_angle){
                       case 90:                        
                        delay(300);
                        myStepper.angle( 188 );
                        delay(300);
                        gl_angle = -90;
                        pattern = 63;                       
                        break;
                       case 180 :
                         delay(300);
                          myStepper.angle(-95);
                          delay(300);
                          gl_angle = 90;
                          pattern = 33;
                          flag10 = 0;
                          break;  
                       case 0:
                          delay(300);
                          myStepper.angle(95);
                          delay(300);
                          gl_angle = 90;
                          pattern = 33;
                          flag10 = 0;
                          break;             
                     }     
                     stepCount = 0;
                     break;
                   case 54 :
                     time1_flag = 0;
                     delay(300);
                     switch( gl_angle ){
                      case 90 :
                         myStepper.angle( 95 );
                         gl_angle = 180 ;
                         pattern = 66;
                        break;
                     case  270 :
                         myStepper.angle( -95 );
                         gl_angle = 180 ; 
                          pattern = 66;                       
                         break;
                     case 180 :
                           myStepper.angle(90);
                           gl_angle = -90;
                          pattern = 63;
                           break;  
                     }                   
                    delay(300);                                     
                     break;
                     /////////
                   case 61 :
                     time1_flag = 1;
                     if( CountLine() ){
                        LineCountY--;
                        stepCount = 0;                       
                     }
                     if( LineCountY == 0){
                         if( stepCount == 65 ) {//50 steps  about 5cm long
                              time1_flag = 0;
                              buz(4);
                              pattern = 2;   // empty  
                         }///   welcome home
                     }
                     break;
                   case 62 :
                     time1_flag = 1;
                     if( CountLine() ){
                        LineCountX--;
                        stepCount = 0;                       
                     }
                     if( LineCountX == 0){
                         if( stepCount == 99 ) {//99 steps  about 10cm long
                              pattern = 61;
                              time1_flag = 0;
                              delay(300);
                              myStepper.angle( - 95 );
                              gl_angle = 90 ;
                             delay(300);     
                             stepCount = 0;
                         }
                     }
                     break;
                   case 63 :
                     time1_flag = 1;
                     if( CountLine() ){
                        LineCountY++;
                        stepCount = 0;
                        step_count_y = 0;
                     }
                     if( LineCountY ==  3){
                         if(stepCount > 149 ) {// 149 steps ,,,about 15cm
                           pattern = 62; //   it is time to turn turn     
                           time1_flag = 0;
                           delay(300);
                           myStepper.angle(95);
                           gl_angle = 180;
                           delay(300);
                           stepCount = 0;
                         }
                     }                        
                     break;
                   case 64:
                     time1_flag = 1;
                     if( CountLine() ){
                        LineCountX--;
                        stepCount = 0;
                        step_count_x = 0;
                     }
                     if( LineCountX ==  2){
                         time1_flag = 0;
                         delay(300);
                         myStepper.angle( 95);
                         gl_angle = -90;
                         delay(300);

                         pattern = 73;
                     }   
                     break;
                   case 65 :  // go straight ahead till the end of the LineY and the turn left
                     time1_flag = 1;
                     if( CountLine() ){
                        LineCountY--;
                        stepCount = 0;
                        step_count_y = 0;
                     }
                     if( LineCountY ==  0){
                         if(stepCount > 149 ) {// 149 steps ,,,about 15cm
                           pattern = 66; //   it is time to turn turn     
                           time1_flag = 0;
                           delay(300);
                           myStepper.angle(95);
                           gl_angle = 180;
                           delay(300);
                         }
                     }   
                     break;
                   case 66:  ///    the last step  
                     time1_flag = 1;
                     if( CountLine() ){
                        LineCountX--;
                        stepCount = 0;
                        step_count_x = 0;
                     }
                     if( LineCountX ==  0){
                         if( stepCount >= 30 )  {// 30 steps about 3cm
                           delay(300);
                           time1_flag = 0;  //   it is time to turn turn  
                           myStepper.angle(95);
                           delay(300);
                           myStepper.step( 260 , 260);
                           delay(1300);
                           myStepper.step( -260 , -260);
                           buz(4);
                           pattern = 2 ; // empty
                         }
                     } 
                     break;
                   case 67 :
                     
                     break;
                     ////////
                   case 71 : // for Y direction transit  ,,and    xx 48 steps for 5 cm  149 steps for 15cm
                     if( stepCount - step_count_y >= xx ){  // 149  
                        pattern = yy; 
                        step_count_y = stepCount;
                     }
                     if( CountLine() ){
                        LineCountY++;
                        stepCount = 0;
                        flag1 = 1;
                     }
                     if ( flag1 ){
                        if( stepCount > ( abs(step_count_y - xx ) ) ) { // when past the Line 
                        pattern = yy;
                        flag1 = 0;
                        step_count_y = stepCount;
                        } 
                     }      
                     break;
                   case 72 : // for X direction transit
                     if( stepCount - step_count_x >= xx ){  //  about 15cm ,,,the lenth of the car
                        pattern = yy; 
                        step_count_x = stepCount;
                     }
                     if( CountLine() ){
                        LineCountX++;
                        stepCount = 0;
                        flag1 = 1;
                     }
                     if ( flag1 ){
                        if( stepCount > ( abs(step_count_x - xx ) ) ){  // when past the Line 
                           pattern = yy;
                           step_count_x = stepCount;
                           flag1 = 0;
                        }
  
                     }      
                     
                     break;
                   case 73 :
                     delay(300);
                     myStepper.step(129 ,129);
                     delay(300);
                     myStepper.angle(-94);
                     delay(300);
                     gl_angle = 180;
                     pattern = 62;
                     step_count_y++;  //very important
                     break;
                   default:
                    /* If neither, return to standby state */
                    pattern = 0;
                    break;
                  case 75 :
                    time1_flag = 1;
                     if( CountLine() ){
                        LineCountX--;
                        stepCount = 0;
                        step_count_x = 0;
                     }
                     if( LineCountX ==  2){
                         
                           pattern = 66; //   it is time to turn turn     
                           time1_flag = 0;
                           delay(300);
                           myStepper.angle( -95);
                           gl_angle = 90;
                           delay(300);
                         
                     } 
                    break;
                  case 76 :
                    if( CountLine() ){
                        LineCountY++;
                        stepCount = 0;                    
                     }
                     if( LineCountY == 2){
                         if( stepCount == 149 ) {//149 steps  about 15cm long
                              time1_flag = 0;
                              delay(300);
                              myStepper.angle( -95);
                              gl_angle = 180;
                              pattern = 62;
                         }///   welcome home
                     }
                    break;
                  case 77 :  // function : the same as case 71,,but in different direction
                    if( stepCount - step_count_y >= xx ){
                        pattern = yy; 
                        step_count_y = stepCount;
                     }
                     if( CountLine() ){
                        LineCountY--;
                        stepCount = 0;
                        flag1 = 1;
                     }
                     if ( flag1 ){
                        if( stepCount > ( abs(step_count_y - xx ) ) ) { // when past the Line 
                        pattern = yy;
                        flag1 = 0;
                        step_count_y = stepCount;
                        } 
                     }      
                    break;
                  case 78 ://// function : the same as case 72,,but in different direction
                     if( stepCount - step_count_x >= xx ){  //  about 15cm ,,,the lenth of the car  
                        pattern = yy; 
                        step_count_x = stepCount;
                     }
                     if( CountLine() ){
                        LineCountX--;
                        stepCount = 0;
                        flag1 = 1;
                     }
                     if ( flag1 ){
                        if( stepCount > ( abs(step_count_x - xx ) ) ){  // when past the Line 
                           pattern = yy;
                           step_count_x = stepCount;
                           flag1 = 0;
                        }  
                     }  
                     break;  
              }
  }
} 
