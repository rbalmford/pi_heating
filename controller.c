// hello

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <bcm2835.h>

int  setup_temp_sensor_1w(char *devPath);
int  read_temp_1w(char *devPath, float *tmeas);
int  setup_ports_bcm();
void signal_callback_handler(int signum);

#define GP_O_ON  25 // GPIO 25
#define GP_I_TDN 15 // GPIO 15
#define GP_I_TUP 14 // GPIO 14

int main() {

  signal(SIGINT, signal_callback_handler);

// constants

  const int8_t tcycle   = 1; // mintes
  const int8_t tmeaspc  = 5;
  const int8_t set_bits = 5;
  const int8_t cyc_bits = 3;

// variables

  int8_t   tset = 19; // degrees
  struct timespec gettime_now;
  long int start_time, next_time, time_now, time_but;
  int8_t   count, state;
  float    tmeas, tacc;
  int8_t   terror, erracc, pprop;
  int8_t   intdis, pinteg, power;
  int8_t   intbit, propbit, mask, bit;
  int8_t   hi_count;
  int8_t   carry;
  int8_t   but_dn, but_up;
  int8_t   init = 1;
  // 1-wire
  char     devPath[128];
  // spi data
  char     spi_out[] = {0x00,0x00};
  // logfile
  struct tm *timeinfo;
  time_t   rawtime;
  char     log_basedir[] = "/home/pi/logs/";
  char     log_timestamp[128];
  char     log_filename[128];
  FILE     *logfile;

// setup I/O

  if (!setup_temp_sensor_1w(devPath))
    exit(1);
  if (!setup_ports_bcm())
    exit(1);

// initial outputs

  bcm2835_gpio_write(GP_O_ON,LOW);
  bcm2835_spi_transfern(spi_out,sizeof(spi_out));

// logfile

  rawtime  = time(NULL);
  timeinfo = localtime(&rawtime); 
  strftime(log_timestamp,128,"%Y-%m-%d_%H-%M-%S",timeinfo); 
  strcpy(log_filename,log_basedir);
  strcat(log_filename,"tlog_");
  strcat(log_filename,log_timestamp);
  strcat(log_filename,".csv");
  printf ("\nlogfile: %s\n",log_filename);
/*
  if ((logfile = fopen(log_filename,"w")) == NULL){
    printf("\nCould not open logfile\n");
    }
  else {
    printf("\nWriting header to logfile\n");
    fprintf(logfile,"date,time,temp,state,prop,int,power,carry,on\n");
    }
*/

  int8_t cycles = (1 << cyc_bits) - 1;
  float  tstate = (float)tcycle * 60.0 / (float)cycles;
  float  tcount = tstate / (float)tmeaspc;

  printf ("\ntcycle: %d tmeaspc: %d",tcycle,tmeaspc);
  printf (" tstate: %f tcount: %f\n",tstate,tcount);

  while (1) {

    if (init) {
      clock_gettime(CLOCK_REALTIME, &gettime_now);
        // times are scaled to units of 10mS
      start_time = (gettime_now.tv_sec*100) + (gettime_now.tv_nsec/10000000);
      count    = 0;
      state    = (2 * cycles) - 1;
      tacc     = 0;
      erracc   = 0;
      intdis   = 1;
      power    = 0;
      hi_count = 0;
      init     = 0;
      }

    if (count != (tmeaspc - 1))
      next_time = start_time + (int)(tcount * (float)(count + 1) * 100.0);
    else {
      next_time  = start_time + (int)(tstate * 100.0);
      start_time = next_time;
      }

    if (!read_temp_1w(devPath, &tmeas))
      exit(1);
    tacc += tmeas;

// state transition

    if (count == 0) {
      if (state < cycles)
        tacc /= (float)tmeaspc;
      terror = (tset << (5 + cyc_bits - set_bits))
               - (int)(tacc * pow(2.0,cyc_bits));
      erracc += terror;
      if (terror > cycles)
        intdis = 1;
      pprop = fmin(cycles,terror);

      state = ++state % cycles;

      if (state == 0) {
//        carry    = fmin(2,fmax(-1,power - hi_count));
        carry    = 0;
        hi_count = 0;
        /**/ if (intdis ==  1)       pinteg  = 0;
        else if (erracc >  2*cycles) pinteg += (pinteg<(cycles-1));
        else if (erracc < -2*cycles) pinteg -= (pinteg>0);;
        }

// PWM

      power    = fmin(cycles,fmax(0,carry + pinteg + pprop));
      intbit   = ((carry + pinteg) > state);
      propbit  = (pprop >= (cycles - state));
      mask     = (power > hi_count);
      bit      = (intbit | propbit) & mask;
      hi_count = hi_count + bit;

      bcm2835_gpio_write(GP_O_ON,(bit ? HIGH : LOW));
      spi_out[0] = (int)(tacc * 2.0);
      spi_out[1] = power;
      bcm2835_spi_transfern(spi_out,sizeof(spi_out));

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

      tacc = 0;
      if (state == 0) {
        erracc = 0;
        intdis = 0;
        }

      } //  end of new state section

    count = ++count % tmeaspc;

    while (1) {
      if (!bcm2835_gpio_lev(GP_I_TDN))
        delay(100);
      while (!bcm2835_gpio_lev(GP_I_TDN)) {
        tset -= (tset>0);
        spi_out[0] = tset<<1;
        bcm2835_spi_transfern(spi_out,sizeof(spi_out));
        printf("%i\n",tset);
        if (!init)
          delay(400);
        init = 1;
        delay(500);
        }
      if (!bcm2835_gpio_lev(GP_I_TUP))
        delay(100);
      while (!bcm2835_gpio_lev(GP_I_TUP)) {
        tset += (tset<30);
        spi_out[0] = tset<<1;
        bcm2835_spi_transfern(spi_out,sizeof(spi_out));
        printf("%i\n",tset);
        if (!init)
          delay(400);
        init = 1;
        delay(500);
        }
      if (init) {
        delay(500);
        break;
        }

      clock_gettime(CLOCK_REALTIME, &gettime_now);
      time_now = (gettime_now.tv_sec*100) + (gettime_now.tv_nsec/10000000);
      if (time_now > next_time)
        break;

      delay(100);
      }

    } // end of while loop

  return EXIT_SUCCESS;

  } // end main

////////////
// functions
////////////

void signal_callback_handler(int signum) {
  printf("\nCaught signal %d\n",signum);
    // Cleanup and close up stuff here
  bcm2835_gpio_write(GP_O_ON,LOW);
  char spi_out[] = {0xaa,0xaa};
  bcm2835_spi_transfern(spi_out,sizeof(spi_out));
  bcm2835_close();
//  fclose(logfile);
    // Terminate program
  exit(signum);
  }

int setup_ports_bcm() {
  if (!bcm2835_init())
    return 0;
// spi
  bcm2835_spi_begin();
  bcm2835_gpio_fsel(RPI_GPIO_P1_26,BCM2835_GPIO_FSEL_OUTP);     // return CE1
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0,LOW);       // the default
// outputs
  bcm2835_gpio_fsel(GP_O_ON,BCM2835_GPIO_FSEL_OUTP);
// inputs
  bcm2835_gpio_fsel(GP_I_TDN,BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(GP_I_TDN,BCM2835_GPIO_PUD_UP);
  bcm2835_gpio_fsel(GP_I_TUP,BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(GP_I_TUP,BCM2835_GPIO_PUD_UP);
  return 1;
  }

int read_temp_1w(char *devPath, float *tmeas) {
  ssize_t numRead;
  char    buf[256];     // Data from device
  char    tmpData[6];   // Temp C * 1000 reported by device 

  int fd = open(devPath, O_RDONLY);
  if(fd == -1) {
    perror ("Couldn't open the w1 device.");
    return 0;
    }
  while((numRead = read(fd, buf, 256)) > 0) {
    strncpy(tmpData, strstr(buf, "t=") + 2, 5);
    *tmeas = strtof(tmpData, NULL) / 1000.0;
    }
  close(fd);
  return 1;
  } // end read_temp

int setup_temp_sensor_1w(char *devPath) {
  struct dirent *dirent;
  DIR           *dir;
  char          dev[16];      // Dev ID
  char          path[] = "/sys/bus/w1/devices"; 

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
    return 0;
    }
  sprintf(devPath, "%s/%s/w1_slave", path, dev);
  return 1;
  } // end setup_temp_sensor

