//Arduino code for generation a PPM signal on pin22, basing values of different joystick potentiometers.
//The aim is to substitute an RC radio and manage the UAV with only one integrated device.
//The code run on an atmel 2560; the board is mounted on a mechanical structure with a tablet in order to obtain an
//integrated GCS (Ground Control Station) able to control a UAV with radio control and receive the telemetry on a integrated PC, managing also the payload.
//The generated PPM signal is sended to a FrSky DJT trasmitter module that communicate with its binded FrSky Receiver.

//The code manage also a fan in order to cooling all the electronic boards in the box.
//In the next the fan depending by the reading of a thermal sensor.

//--->CONFIGURATION
#define chanel_number 8           
#define default_servo_value 1500  
#define PPM_FrLen 22500           
#define PPM_PulseLen 300          
#define onState 1                 
#define sigPin 22                 

//--->Flight mode PWM
#define mode1 1000
#define mode2 1400
#define mode3 1680
#define mode4 1799

const int yaw = A0;
const int th = A1;
const int pitch = A6;
const int roll = A7;
const int pay1 = A5;
const int pay2 = A6;

const int enable_sig = 19;

int sv_th = 0;
int sv_roll = 0;
int sv_yaw = 0;
int sv_pitch = 0;
int sv_pay1 = 0;
int sv_pay2 = 0;

int out_th = 0;
int out_roll = 0;
int out_yaw = 0;
int out_pitch = 0;
int out_pay1 = 0;
int out_pay2 = 0;
int sw1=0;
int sw2=0;
int sw3=0;
int sw4=0;
int sw5=0;
int sw6=0;
int sw7=0;
int sw8=0;

int old_sw1=0;
int old_sw2=0;
int old_sw3=0;
int old_sw4=0;
int old_sw5=0;
int old_sw6=0;
int old_sw7=0;
int old_sw8=0;

int i=0;
int ch5=1000;

int ppm[chanel_number]; //Vector to store value of each signal

void setup(){  
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  analogWrite(18,130);
  digitalWrite(sigPin, !onState);
  
  cli();
  TCCR1A = 0; 
  TCCR1B = 0;
  OCR1A = 100;              
  TCCR1B |= (1 << WGM12);   
  TCCR1B |= (1 << CS11);   
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void loop(){
  if(sw7){
    analogWrite(enable_sig,255);
    ppm[0]=1528;          
    ppm[1]=1485;          
    ppm[2]=1490;          
    ppm[3]=1518;          
  }
  else{
    analogWrite(enable_sig,0);

    i++;
    old_sw2=sw2;
    old_sw3=sw3;
    old_sw4=sw4;
    
    
    sv_roll= analogRead(roll);
    sv_pitch= analogRead(pitch);
    sv_pay1=analogRead(pay1);
    sv_pay2=analogRead(pay2);
    sw1=digitalRead(13);
    sw2=digitalRead(12);
    sw3=digitalRead(11);
    sw4=digitalRead(10);
    sv_th = analogRead(th);
    sv_yaw= analogRead(yaw);
    sw5=digitalRead(17);
    sw6=digitalRead(15);
    sw7=digitalRead(14);
    sw8=digitalRead(16);
  
  
    //MAPPING
    out_th = map(sv_th, 2, 896, 1000, 2000);
    out_roll = map(sv_roll, 85, 997, 1000, 2000);
    out_yaw = map(sv_yaw, 51, 990, 1000, 2000);
    out_pitch = map(sv_pitch, 71, 995, 1000, 2000);
    out_pay1=map(sv_pay1,0,1023,1000,2000);

    ppm[0]=out_roll;   
    ppm[1]=out_pitch;   
    ppm[2]=out_th;      
    ppm[3]=out_yaw;     
    
    if(sw2 && old_sw2!=sw2){
      ch5=mode1;
    }
    else if(sw3 && old_sw3!=sw3){
      ch5=mode2;
    }
    else if(sw4 && old_sw4!=sw4){
      ch5=mode3;
    }
    else if(sw5 && old_sw5!=sw5){
      ch5=mode4;
    }
    
    ppm[4]=ch5;
    ppm[5]=out_pay1;
    
    if(sw6){
      ppm[6]=1900;
    }
    else{
      ppm[6]=1100;
    }
   }
  delay(1);
}

ISR(TIMER1_COMPA_vect){
  static boolean state = true;
  TCNT1 = 0;
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
