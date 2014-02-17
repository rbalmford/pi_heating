// hello

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "gb_common.h"

int  setup_temp_sensor(char *devPath);
int  read_temp(char *devPath, float *tmeas);
void setup_ports();

#define GPIO_BIT  7 // GPIO  7
#define GPIO_TDN 25 // GPIO 25
#define GPIO_TUP 27 // GPIO 27

int main() {

// constants

  const int tcycle   = 1; // mintes
  const int tmeaspc  = 5;
  const int set_bits = 5;
  const int cyc_bits = 3;

// variables

  int   tset = 20; // degrees

  char  devPath[128];
  int   count, state;
  float tmeas, tacc;
  int   terror, erracc, intdis;
  int   pinteg, pprop, power;
  int   intbit, propbit, mask, bit;
  int   hi_count, carry;
  int   init = 1;

  long int        start_time, next_time, time_now;
  struct timespec gettime_now;

  struct tm *timeinfo;
  time_t    rawtime;
  char      *log_basedir = "/home/pi/logs/";
  char      log_timestamp[128];
  char      log_filename[128];
  FILE      *logfile;

// setup

  setup_temp_sensor(devPath);

  setup_io();
  setup_ports();
  GPIO_CLR0 = (1 << GPIO_BIT);

// GPIO event triggers
//

// open logfile

  rawtime  = time(NULL);
  timeinfo = localtime(&rawtime); 
  strftime(log_timestamp,128,"%Y-%m-%d_%H-%M-%S",timeinfo); 

  strcpy(log_filename, log_basedir);
  strcat(log_filename, "tlog_");
  strcat(log_filename, log_timestamp);
  strcat(log_filename, ".csv");

  printf ("\nlogfile: %s\n",log_filename);
/*
  if((logfile = fopen(log_filename,"w")) == NULL){
    printf("\nCould not open logfile\n");
    };

  fprintf(logfile,"date,time,temp,state,prop,int,power,carry,on\n");

  fclose(logfile);
*/
  int   cycles = (1 << cyc_bits) - 1;
  float tstate = (float)tcycle * 60.0 / (float)cycles;
  float tcount = tstate / (float)tmeaspc;

  printf ("\ntcycle: %d tmeaspc: %d",tcycle,tmeaspc);
  printf (" tstate: %f tcount: %f\n",tstate,tcount);

  clock_gettime(CLOCK_REALTIME, &gettime_now);
// times are scaled to units of 10mS
  start_time = (gettime_now.tv_sec*100) + (gettime_now.tv_nsec/10000000);

  while (1) {

    if (init) {
      count    = 0;
      state    = (2 * cycles) - 1;
      tacc     = 0;
      erracc   = 0;
      intdis   = 1;
      power    = 0;
      hi_count = 0;
      init     = 0;
      }

    if (count == (tmeaspc - 1)) {
      next_time  = start_time + (int)(tstate * 100.0);
      start_time = next_time;
      }
    else
      next_time = start_time + (int)(tcount * (float)(count + 1) * 100.0);
//    printf("\nstart_time: %li  next_time: %li",start_time,next_time);

    read_temp(devPath, &tmeas);
//    tmeas = 17.5;
    tacc += tmeas;

// state transition

    if (count == 0) {
      if (state < cycles)
        tacc /= (float)tmeaspc;

      state = ++state % cycles;

      terror = (tset << (5 + cyc_bits - set_bits))
               - (int)(tacc * pow(2.0,cyc_bits));

      if (terror > cycles)
        intdis = 1;
      erracc += terror;

// proportional path

      pprop = fmin(cycles,terror);

      if (state == 0) {
        carry    = fmin(2,fmax(-1,power - hi_count));
        hi_count = 0;

// integral path

        /**/ if (intdis ==  1)        pinteg  = 0;
        else if (erracc >=  5*cycles) pinteg += 2;
        else if (erracc >=  2*cycles) pinteg += 1;
        else if (erracc <= -5*cycles) pinteg -= 2;
        else if (erracc <= -2*cycles) pinteg -= 1;
        pinteg = fmin(cycles-1,fmax(0,pinteg));
        }

// PWM

      power    = fmin(cycles,fmax(0,carry + pinteg + pprop));
      intbit   = ((carry + pinteg) > state);
      propbit  = (pprop >= (cycles - state));
      mask     = (power > hi_count);
      bit      = (intbit | propbit) & mask;
      hi_count = hi_count + bit;

      printf ("\n");
      printf ("state: %d", state);
      printf (" tset: %d", tset);
      printf (" tave: %.3f", tacc);
      printf (" tave_r: %.3f", (int)(tacc*pow(2.0,cyc_bits))/pow(2.0,cyc_bits));
      printf (" terror: %d", terror);
      printf ("\nerracc: %d", erracc);
      printf (" intdis: %d", intdis);
      printf (" carry: %d", carry);
      printf (" pinteg: %d", pinteg);
      printf (" pprop: %d", pprop);
      printf (" power: %d", power);
      printf (" bit: %d", bit);
      printf (" hi_c: %d", hi_count);
      printf ("\n");

      if (bit)
        GPIO_SET0 = (1 << GPIO_BIT);
      else
        GPIO_CLR0 = (1 << GPIO_BIT);

      tacc = 0;
      if (state == 0) {
        erracc = 0;
        intdis = 0;
        }

      } //  end of new state section

    count = ++count % tmeaspc;

    do {
      sleep(0.1);
      clock_gettime(CLOCK_REALTIME, &gettime_now);
      time_now = (gettime_now.tv_sec*100) + (gettime_now.tv_nsec/10000000);
      }
    while ((time_now < next_time) && (!init));

    } // end of while loop

  } // end main

////////////
// functions
////////////

void setup_ports() {
  INP_GPIO(GPIO_BIT); OUT_GPIO(GPIO_BIT);
  INP_GPIO(GPIO_TDN);
  INP_GPIO(GPIO_TUP);

  GPIO_PULL = 2;  // 0: dis  1: PDN  2: PUP
  short_wait();
  GPIO_PULLCLK0 = ((1 << GPIO_TDN) | (1 << GPIO_TUP));
  short_wait();
  GPIO_PULL = 0;  // 0: dis  1: PDN  2: PUP
  GPIO_PULLCLK0 = 0;
  }

int read_temp(char *devPath, float *tmeas) {
  ssize_t numRead;
  char    buf[256];     // Data from device
  char    tmpData[6];   // Temp C * 1000 reported by device 

  int fd = open(devPath, O_RDONLY);

  if(fd == -1) {
    perror ("Couldn't open the w1 device.");
    return 1;
    }
  while((numRead = read(fd, buf, 256)) > 0) {
    strncpy(tmpData, strstr(buf, "t=") + 2, 5);
    *tmeas = strtof(tmpData, NULL) / 1000.0;
    }
  close(fd);
  return 0;
  } // end read_temp

int setup_temp_sensor(char *devPath) {
  struct dirent *dirent;
  DIR     *dir;
  char    dev[16];      // Dev ID
  char    path[] = "/sys/bus/w1/devices"; 

  dir = opendir (path);
  if (dir != NULL) {
    while ((dirent = readdir (dir)))
      if (dirent->d_type == DT_LNK && 
          strstr(dirent->d_name, "28-") != NULL) { 
        strcpy(dev, dirent->d_name);
        printf("\nfound device: %s\n", dev);
        }
    (void) closedir (dir);
    }
  else {
    perror ("Couldn't open the w1 devices directory");
    return 1;
    }
  sprintf(devPath, "%s/%s/w1_slave", path, dev);
  return 0;
  } // end setup_temp_sensor

