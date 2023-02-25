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

int fd=-1;

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

int open_serial_port(char *serial_port)
{
  int serial_speed=115200;
  
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

int left=0,right=0;

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
  if (n==26) {
    // Got complete line
    int lvalue=s[left];
    int rvalue=s[right];
    printf("l=%d,r=%d\n",lvalue,rvalue);
  }
}

int main(int argc,char **argv)
{
  if (argc!=4) {
    fprintf(stderr,"usage: mouth_force_reader <serialport> <left sensor num> <right sensor num>\n");
    exit(-1);
  }
  char *serial_port=argv[1];
  left=atoi(argv[2]);
  right=atoi(argv[3]);
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

  open_serial_port(serial_port);

  char line[1024];
  int len=0;
  while(1) {
    char buf[8192];
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
