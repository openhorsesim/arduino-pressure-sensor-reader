/*
  Copyright (C) 2014-2023 Paul Gardner-Stephen
  Portions Copyright (C) 2021-2023 MEGA65 Project Contributors
  Portions Copyright (C) 2013 Serval Project Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

*/


// Set minimum measurement range.
// Random noise of about +/- 5 to 10 is normal.
// Set too low, then dynamic range changes with low pressure causes weirdness.
// Set too high, it becomes too insensitive "heavy in the mouth"
#define MIN_RANGE 200

#define SHOW_FORCES
// #define SHOW_RATIOS

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <strings.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <linux/tty_flags.h>
#include <math.h>

int fd=-1;
int mousefd=-1;
int outfd=-1;

#define log_error(...) { fprintf(stderr,"ERROR: "); fprintf(stderr,__VA_ARGS__); fprintf(stderr,"\n"); }
#define log_crit(...) { fprintf(stderr,"CRITICAL: "); fprintf(stderr,__VA_ARGS__); fprintf(stderr,"\n"); }
#define log_warn(...) { fprintf(stderr,"WARNING: "); fprintf(stderr,__VA_ARGS__); fprintf(stderr,"\n"); }
#define log_info(...) { fprintf(stderr,"INFO: "); fprintf(stderr,__VA_ARGS__); fprintf(stderr,"\n"); }
#define log_debug(...) { fprintf(stderr,"DEBUG: "); fprintf(stderr,__VA_ARGS__); fprintf(stderr,"\n"); }

void set_serial_speed(int fd, int serial_speed)
{
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, NULL) | O_NONBLOCK);
  struct termios t;

  if (fd < 0) {
    log_error("set_serial_speed: invalid fd");
    return;
  }

#ifdef __APPLE__
  /*
   * This code is needed because recent versions of MacOS do not allow
   * setting 'strange' baud rates (like 2000000) via tcsetattr().
   */
  speed_t speed = serial_speed;
  log_debug("set_serial_speed: %d bps (OSX)", serial_speed);
  if (ioctl(fd, IOSSIOSPEED, &speed) == -1) {
    log_error("failed to set output baud rate using IOSSIOSPEED");
  }
  if (tcgetattr(fd, &t))
    log_error("failed to get terminal parameters");
  cfmakeraw(&t);
  // TCSASOFT prevents some fields (most importantly the baud rate)
  // from being changed.  MacOS does not support 'strange' baud rates
  // that might be set by the ioctl above.
  if (tcsetattr(fd, TCSANOW | TCSASOFT, &t))
    log_error("failed to set OSX terminal parameters");

  // Serial port will be unresponsive after returning from FREEZER
  // without this.
  tcflush(fd, TCIFLUSH);
#else
  log_debug("set_serial_speed: %d bps (termios)", serial_speed);
  if (serial_speed == 115200) {
    if (cfsetospeed(&t, B115200))
      log_error("failed to set output baud rate");
    if (cfsetispeed(&t, B115200))
      log_error("failed to set input baud rate");
  }
  else if (serial_speed == 230400) {
    if (cfsetospeed(&t, B230400))
      log_error("failed to set output baud rate");
    if (cfsetispeed(&t, B230400))
      log_error("failed to set input baud rate");
  }
  else if (serial_speed == 2000000) {
    if (cfsetospeed(&t, B2000000))
      log_error("failed to set output baud rate");
    if (cfsetispeed(&t, B2000000))
      log_error("failed to set input baud rate");
  }
  else if (serial_speed == 1000000) {
    if (cfsetospeed(&t, B1000000))
      log_error("failed to set output baud rate");
    if (cfsetispeed(&t, B1000000))
      log_error("failed to set input baud rate");
  }
  else if (serial_speed == 1500000) {
    if (cfsetospeed(&t, B1500000))
      log_error("failed to set output baud rate");
    if (cfsetispeed(&t, B1500000))
      log_error("failed to set input baud rate");
  }
  else {
    log_warn("Defaulting to 4mbps");
    if (cfsetospeed(&t, B4000000))
      log_error("failed to set output baud rate");
    if (cfsetispeed(&t, B4000000))
      log_error("failed to set input baud rate");
  }

  t.c_cflag &= ~PARENB;
  t.c_cflag &= ~CSTOPB;
  t.c_cflag &= ~CSIZE;
  t.c_cflag &= ~CRTSCTS;
 t.c_cflag |= CS8 | CLOCAL;
  t.c_lflag &= ~(ICANON | ISIG | IEXTEN | ECHO | ECHOE);
  t.c_iflag &= ~(BRKINT | ICRNL | IGNBRK | IGNCR | INLCR | INPCK | ISTRIP | IXON | IXOFF | IXANY | PARMRK);
  t.c_oflag &= ~OPOST;
  if (tcsetattr(fd, TCSANOW, &t))
    log_error("failed to set terminal parameters");

  // Also set USB serial port to low latency
  struct serial_struct serial;
  ioctl(fd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(fd, TIOCSSERIAL, &serial);
#endif
}

void close_serial_port(void)
{
  close(fd);
}

int open_serial_port(char *serial_port,int serial_speed)
{
  
errno = 0;
  fd = open(serial_port, O_RDWR);
  if (fd == -1) {
    log_crit("could not open serial port '%s'", serial_port);
    return -1;
  }

  set_serial_speed(fd, serial_speed);

#ifdef __linux__
  // Also try to reduce serial port latency on linux
  char *last_part = serial_port;
  for (int i = 0; serial_port[i]; i++)
    if (serial_port[i] == '/')
      last_part = &serial_port[i + 1];

  char latency_file[1024];
  snprintf(latency_file, 1024, "/sys/bus/usb-serial/devices/%s/latency_timer", last_part);
  FILE *f = fopen(latency_file, "r");
  if (f) {
    char line[1024];
    fread(line, 1024, 1, f);
    int latency = atoi(line);
    fclose(f);
    if (latency != 1) {
      f = fopen(latency_file, "w");
      if (!f) {
        log_warn("cannot write to '%s' to reduce USB port latency. Performance will be reduced.", latency_file);
        log_warn("  You can try something like the following to fix it:");
        log_warn("    echo 1 | sudo tee %s\n", latency_file);
      }
      else {
        fprintf(f, "1\n");
        fclose(f);
        log_info("reduced USB latency from %d ms to 1 ms.", latency);
      }
    }
  }
#endif /* __linux__ */

  return 0;
}

#define MOUTH_HISTORY_LEN 128
int lhistory[MOUTH_HISTORY_LEN];
int rhistory[MOUTH_HISTORY_LEN];
int count=0;
int lmin=0, lmax=0;
int rmin=0, rmax=0;
int lmean=0, rmean=0;
int loffset=0, roffset=0;

float prev_ratio=1;

void update_mouth(int l,int r, int c)
{
  for(int i=0;i<(MOUTH_HISTORY_LEN-1);i++) {
    lhistory[i]=lhistory[i+1];
    rhistory[i]=rhistory[i+1];
  }
  lhistory[MOUTH_HISTORY_LEN-1]=l;
  rhistory[MOUTH_HISTORY_LEN-1]=r;

  int lsum=0;
  int rsum=0;
  int sumc=0;
  
  lmin=lhistory[MOUTH_HISTORY_LEN-1-count];
  lmax=lhistory[MOUTH_HISTORY_LEN-1-count];
  rmin=rhistory[MOUTH_HISTORY_LEN-1-count];
  rmax=rhistory[MOUTH_HISTORY_LEN-1-count];
  for(int i=(MOUTH_HISTORY_LEN-count);i<MOUTH_HISTORY_LEN;i++) {
    lsum+=lhistory[i];
    rsum+=rhistory[i];
    sumc++;
    if (lhistory[i]<lmin) lmin=lhistory[i];
    if (lhistory[i]>lmax) lmax=lhistory[i];
    if (rhistory[i]<rmin) rmin=rhistory[i];
    if (rhistory[i]>rmax) rmax=rhistory[i];
  }
  if (sumc) {
    lmean=lsum/sumc;
    rmean=rsum/sumc;
  }

  //  printf(">> %d %d %d\n",l,lmean,loffset);	 
  if (l<lmean) loffset--;
  if (l>lmean) loffset++;
  if (r<rmean) roffset--;
  if (r>rmean) roffset++;

  if (loffset<-100) loffset=-100;
  if (loffset>100) loffset=100;
  if (roffset<-100) roffset=-100;
  if (roffset>100) roffset=100;
  
  if (count<(MOUTH_HISTORY_LEN-1)) count++;

  int lrange=lmax-lmin;
  int rrange=rmax-rmin;
  if (lrange<MIN_RANGE) lrange=MIN_RANGE;
  if (rrange<MIN_RANGE) rrange=MIN_RANGE;

#if 0
  // XXX Not really working code that was supposed to relax bias when idle
  int lv=l+loffset;
  if (lv>lmax) lv=lmax;
  if (lv<lmin) lv=lmin;
  int rv=r+roffset;
  if (rv>rmax) rv=rmax;
  if (rv<rmin) rv=rmin;
#else
  int lv=l;
  int rv=r;
#endif
  int cv=c;
  
  float lpercent=100-100.0*(lmax-lv)/lrange;
  float rpercent=100-100.0*(rmax-rv)/rrange;

  float ratio=log((1.0+lmax-lv)/(1.0+rmax-rv))/log(2.0);
#ifdef SHOW_RATIOS
  int m = (ratio-prev_ratio)*4+32;
  if (m<0) m=0; if (m>63) m=63;
  for(int i=0;i<64;i++) {
    if (m<i) printf("#"); else printf(" ");
  }
  printf("  :: change=%f\n",ratio-prev_ratio);
#endif  
  prev_ratio=ratio;
  
  // Compute apparent direction based on ratio of L:R force
  // All L = -1, all R = +1
  float sum=lpercent+rpercent;
  if (!sum) sum=1;
  float lbias=lpercent/sum;
  float rbias=rpercent/sum;
  float bias=rbias-lbias;

#if 0
  printf("direction=%.1f, l=%.1f%% %d(%d..%d)/%d,r=%.1f%% %d(%d..%d)/%d, count=%d, ol=%d\n",
	 bias,
	 lpercent,lv,lmin,lmax,lmean,
	 rpercent,rv,rmin,rmax,rmean,
	 count,
	 lhistory[MOUTH_HISTORY_LEN-count]);
  printf("sum=%.1f, lbias=%.1f, rbias=%.1f\n",sum,lbias,rbias);
#endif

#ifdef SHOW_FORCES
  printf("[");
  float tforce=(lpercent+rpercent)/2;
  for(int i=0;i<32;i++) {
      float t = 100.0-100.0*i/32.0;
      if (tforce<t) printf("#"); else printf(" ");
  }
  printf("]  :  [");
  
  if (bias>0) {
    for(int i=0;i<32;i++) {
      float t = 1.0-1.0*i/32.0;
      if (bias<t) printf(" "); else printf("<");
    }
    printf("                                ");    
  } else {
    printf("                                ");    
    for(int i=0;i<32;i++) {
      float t = 1.0*i/32.0;
      t=-t;
      if (bias>t) printf(" "); else printf(">");
    }
  }

  printf("]  :  %d/%d, %d/%d %d/%d/%d",
	 (lmax-lv),lrange,
	 (rmax-rv),rrange,
	 loffset,1024-cv,roffset
	 );
  printf("\n");
#endif
  
}

int left=0,right=0, centre=0;

int parse_line(char *line)
{
  int msec,samples,s[24];
  int n=sscanf(line,"T+%d: x%d: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
	       &msec,&samples,
	       &s[0+0],&s[0+1],&s[0+2],&s[0+3],
	       &s[4+0],&s[4+1],&s[4+2],&s[4+3],
	       &s[8+0],&s[8+1],&s[8+2],&s[4+3],
	       &s[12+0],&s[12+1],&s[12+2],&s[12+3],
	       &s[16+0],&s[16+1],&s[16+2],&s[16+3],
	       &s[20+0],&s[20+1],&s[20+2],&s[20+3]
	       );

  //  printf("parsed %d fields: msec=%d, samples=%d\n",n,msec,samples);  
  //  for(int i=0;i<24;i++) printf("s[%d]=%d, ",i,s[i]);
  //  printf("\n");
  if (n>=6) {
    // Got complete line
    int lvalue=s[left];
    int rvalue=s[right];
    int cvalue=s[centre];
    update_mouth(lvalue,rvalue,cvalue);
  }
  return 0;
}

int open_mouse(char *path)
{
  errno = 0;
  mousefd = open(path, O_RDONLY);
  if (mousefd == -1) {
    log_crit("could not open mouse input device '%s'", path);
    return -1;
  }
  fcntl(mousefd, F_SETFL, fcntl(mousefd, F_GETFL, NULL) | O_NONBLOCK);
  
  return 0;
}

unsigned long long gettime_ms()
{
  struct timeval nowtv;
  // If gettimeofday() fails or returns an invalid value, all else is lost!
  if (gettimeofday(&nowtv, NULL) == -1)
    perror("gettimeofday");
  return nowtv.tv_sec * 1000LL + nowtv.tv_usec / 1000;
}

void key_press(unsigned char c)
{
  char msg[16];
  snprintf(msg,16,"\r\n5%02x\r\n",c);
  int len=strlen(msg);
  int w=write(outfd,msg,strlen(msg));
  fprintf(stderr,"DEBUG: Wrote %d of %d bytes: '%s'\n",w,len,msg);
}

void key_release(unsigned char c)
{
  char msg[16];
  snprintf(msg,16,"\r6%02x\r",c);
  int len=strlen(msg);
  int w=write(outfd,msg,strlen(msg));
  fprintf(stderr,"DEBUG: Wrote %d of %d bytes: '%s'\n",w,len,msg);
}

int main(int argc,char **argv)
{
  if (argc!=7) {
    fprintf(stderr,"usage: mouth_force_reader <serialport> <mouse port> <arduino port> <left sensor num> <right sensor num> <centre sensor num>\n");
    exit(-1);
  }
  char *serial_port=argv[1];
  char *mouse_port=argv[2];
  char *arduino_port=argv[3];
  left=atoi(argv[4]);
  right=atoi(argv[5]);
  centre=atoi(argv[6]);
  if (left<1||left>24) {
    log_error("Left sensor value must be in range 1 -- 24");
    exit(-1);
  }
  if (right<1||right>24) {
    log_error("Right sensor value must be in range 1 -- 24");
    exit(-1);
  }
  // Make left and right relative to 0
  left--; right--;

  open_mouse(mouse_port);
  open_serial_port(arduino_port,9600);
  outfd=fd;
  open_serial_port(serial_port,115200);

  int mouse_x=0;
  int mouse_x_min=-1;
  int mouse_x_max=1;
  float mouse_x_percent=50;
  float mouse_x_percent_last=50;
  int turn_rate_ms=0;
  int turn_active=0;

  unsigned long long turn_pwm_ms=0;
  
  char line[1024];
  int len=0;
  long long last_time=0;
  while(1) {

    // Don't burn all CPU time
    // XXX - Make more efficient using event triggering
    usleep(1);
    
    // Work out where we are in the 1-second PWM cycle
    long long now=gettime_ms();
    long long elapsed=now-last_time;
    last_time=now;
    if (elapsed>1000) elapsed=1000;
    turn_pwm_ms+=elapsed;

    //    fprintf(stderr,"DEBUG: turn_pwm_ms=%d, turn_rate_ms=%d\n",turn_pwm_ms,turn_rate_ms);
    
    // Implement the 
    if ((!turn_pwm_ms)||(turn_pwm_ms>999)) {
      turn_pwm_ms=0;
      if (mouse_x_percent<45) {
	// release 'd' and press 'a'
	if (!turn_active) {
	  fprintf(stderr,"DEBUG: turn LEFT on\n");
	  key_press(0x61);
	  turn_active=1;
	}
      } else if (mouse_x_percent>55) {
	// release 'a' and press 'd'
	if (!turn_active) {
	  fprintf(stderr,"DEBUG: turn RIGHT on\n");
	  key_press(0x64);
	  turn_active=1;
	}
      } else {
	// Cancel turn: release 'a' and 'd' keys
	if (turn_active) {
	  fprintf(stderr,"DEBUG: turn --\n");
	  key_release(0x64); key_release(0x61);
	  turn_active=0;
	}
      }
    } else {
      if (turn_pwm_ms>=turn_rate_ms) {
	// Cancel turn: release 'a' and 'd' keys
	if (turn_active) {
	fprintf(stderr,"DEBUG: turn PWM end\n");
	key_release(0x64); key_release(0x61);
	turn_active=0;
	}
      }
    }
    
    char buf[8192];
    int mn=read(mousefd,buf,3);
    if (mn==3) {
      int delta_x=buf[1];
      mouse_x+=delta_x;
      if (mouse_x<mouse_x_min) mouse_x_min=mouse_x;
      if (mouse_x>mouse_x_max) mouse_x_max=mouse_x;
      mouse_x_percent_last=mouse_x_percent;
      mouse_x_percent=100.0*(mouse_x-mouse_x_min)/(mouse_x_max-mouse_x_min+1);
      fprintf(stderr,"DEBUG: Mouse X delta=%d, Mouse X = %d = %.1f%% of travel, range=%d..%d\n",
	      delta_x,mouse_x,mouse_x_percent,mouse_x_min,mouse_x_max);
      turn_rate_ms=0;
#define MIN_KEY_DURATION_MS 100.0
      if (mouse_x_percent>55) {
	turn_rate_ms=(mouse_x_percent-55)/45.0*(1000.0-MIN_KEY_DURATION_MS);
      } else if (mouse_x_percent<45) {
	turn_rate_ms=(45-mouse_x_percent)/45.0*(1000.0-MIN_KEY_DURATION_MS);
      }
      fprintf(stderr,"DEBUG: turn PWM = %dms\n",turn_rate_ms);
    }
    int n=read(fd,buf,8192);
    if (n>0) {
      for(int o=0;o<n;o++) {
	char c = buf[o];
	if (c==0x0a||c==0x0d) {
	  // End of line
	  if (len) {
	    //	    log_debug("Read line '%s'",line);
	    if (line[0]=='T') {
	      // It's probably a complete line
	      parse_line(line);
	    }
	    len=0;
	  }
	} else {
	  // Accumulate line
	  if (len<1024) {
	    line[len++]=c;
	    line[len]=0;
	  }
	}
      }
    }
  }
  
  return 0;
}
