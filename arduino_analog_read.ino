void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

// Set control pins to output
  for(int i=4;i<12;i++) pinMode(i,OUTPUT);

}

#define ANALOG_PINS 6
#if ANALOG_PINS == 6
// XXX Tuned for 50Hz
// XXX Make dynamic using a PLL or similar logic to track the mains frequency?
// (but it still won't deal with other electrical noise)
#define MAX_COUNT 29
#else
// XXX - Put in values for each ANALOG_PINS value
#define MAX_COUNT 512
#endif

// HISTORY should be a power of 2 for performance
#define HISTORY 4
  long sum[ANALOG_PINS*4]={0};
  int recent[ANALOG_PINS*4][HISTORY]={0};
  int recent_sum[ANALOG_PINS*4]={0};
  int avg[ANALOG_PINS*4]={0};
  // 4-way multiplexor on analog inputs
  int d[ANALOG_PINS*4];
  int count=0;
  int zero_crossed=1;

  int last=0;
  int rising=0;

  int historesis=4;

  int m=0,last_m=0;

void report_averages(int fullP)
{
        m = millis()-last_m;
        Serial.write("T+");
        Serial.print(m);
        Serial.write(": x");
        Serial.print(count);
        Serial.write(": ");
        
        // Assume we have hit the peak
        for(int a=0;a<ANALOG_PINS*4;a++) {
          avg[a]=sum[a]/count;

          recent_sum[a]-=recent[a][0];
          recent_sum[a]+=avg[a];
          for(int i=0;i<(HISTORY-1);i++) recent[a][i]=recent[a][i+1];
          recent[a][HISTORY-1]=avg[a];

          Serial.print(recent_sum[a]/HISTORY);
          Serial.write(","); 
          sum[a]=0; 
        }
        if (fullP) Serial.write("   NO ZERO CROSSING");
        Serial.write("\r\n"); 
        last_m=millis();    
}

void resistorSelect(int KOhms)
{
  if (KOhms==10000) digitalWrite(4,1); else digitalWrite(4,0);
  if (KOhms==1000)  digitalWrite(5,1); else digitalWrite(5,0);
  if (KOhms==100)   digitalWrite(6,1); else digitalWrite(6,0);
  if (KOhms==10)    digitalWrite(7,1); else digitalWrite(7,0);

}

void sensorSelect(int s)
{
  if (s==1) digitalWrite(8,1); else digitalWrite(8,0);
  if (s==2) digitalWrite(9,1); else digitalWrite(9,0);
  if (s==3) digitalWrite(10,1); else digitalWrite(10,0);
  if (s==4) digitalWrite(11,1); else digitalWrite(11,0);
}


void loop() {
  // put your main code here, to run repeatedly:

  for(int a=0;a<ANALOG_PINS;a++) {
     recent_sum[a]=0;
     for(int h=0;h<HISTORY;h++) recent[a][h]=0;
  }

  // XXX - Temporarily select resistor value and analog line 
  // from multiplexed analog input shield


  // Select 10M Ohm resistor
  resistorSelect(10000);

  // Select sensor 1 of 4 on each analog input
  sensorSelect(1);

  while(1) {
    for(int sens=0;sens<4;sens++) {
      sensorSelect(sens+1);
    // Read all analog pins
    d[0+sens]=analogRead(A0);
#if ANALOG_PINS > 1 
    d[4+sens]=analogRead(A1);
#endif
#if ANALOG_PINS > 2
    d[8+sens]=analogRead(A2);
#endif
#if ANALOG_PINS > 3
    d[12+sens]=analogRead(A3);
#endif
#if ANALOG_PINS > 4
    d[16+sens]=analogRead(A4);
#endif
#if ANALOG_PINS > 5
    d[20+sens]=analogRead(A5);
#endif
    }

    if (d[0]<last) {
      if (rising==-(historesis-1)&&zero_crossed&&(count>20)) {
        report_averages(0);
        count=0;
        zero_crossed=0;
      }
      if (rising>-historesis) {
        rising--;
      }
    } else if (d[0]>last) {
      if (rising<historesis) rising++;
      if (rising==historesis) zero_crossed=1;
    }
    if (count<MAX_COUNT) {
      count++; 
      for(int a=0;a<ANALOG_PINS*4;a++) sum[a]+=d[a];
    } else {
      report_averages(1);
      count=0;
    }
    last=d[0];

  }
}
