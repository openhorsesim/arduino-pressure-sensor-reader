void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
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
  long sum[ANALOG_PINS]={0};
  int recent[ANALOG_PINS][HISTORY]={0};
  int recent_sum[ANALOG_PINS]={0};
  int avg[ANALOG_PINS]={0};
  int d[ANALOG_PINS];
  int count=0;
  int zero_crossed=1;

  int last=0;
  int rising=0;

  int historesis=4;

  int m=0,last_m=0;

void report_averages(void)
{
        m = millis()-last_m;
        Serial.write("T+");
        Serial.print(m);
        Serial.write(": x");
        Serial.print(count);
        Serial.write(": ");
        
        // Assume we have hit the peak
        for(int a=0;a<ANALOG_PINS;a++) {
          avg[a]=sum[a]/count;

          recent_sum[a]-=recent[a][0];
          recent_sum[a]+=avg[a];
          for(int i=0;i<(HISTORY-1);i++) recent[a][i]=recent[a][i+1];
          recent[a][HISTORY-1]=avg[a];

          Serial.print(recent_sum[a]/HISTORY);
          Serial.write(","); 
          sum[a]=0; 
        }
        Serial.write("\r\n"); 
        last_m=millis();    
}


void loop() {
  // put your main code here, to run repeatedly:

  for(int a=0;a<ANALOG_PINS;a++) {
     recent_sum[a]=0;
     for(int h=0;h<HISTORY;h++) recent[a][h]=0;
  }

  while(1) {
    // Read all analog pins
    d[0]=analogRead(A0);
#if ANALOG_PINS > 1 
    d[1]=analogRead(A1);
#endif
#if ANALOG_PINS > 2
    d[2]=analogRead(A2);
#endif
#if ANALOG_PINS > 3
    d[3]=analogRead(A3);
#endif
#if ANALOG_PINS > 4
    d[4]=analogRead(A4);
#endif
#if ANALOG_PINS > 5
    d[5]=analogRead(A5);
#endif

    if (d[0]<last) {
      if (rising==-(historesis-1)&&zero_crossed&&(count>20)) {
        report_averages();
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
      for(int a=0;a<ANALOG_PINS;a++) sum[a]+=d[a];
    } else {
      Serial.write("FULL: T+");
      report_averages();
      count=0;
    }
    last=d[0];

  }
}
