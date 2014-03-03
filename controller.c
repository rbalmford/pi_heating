////// hello

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
#include <wiringSerial.h>
#include <errno.h>

int8_t setup_temp_sensor_1w(char *devPath);
int8_t read_temp_1w(char *devPath, float *tmeas);
void   setup_ports_bcm();
void   setup_i2c_bcm();
void   setup_spi_bcm();
int8_t read_temp_i2c(float *tmeas);
int8_t i2c_ascii(char val);
int8_t i2c_curpos(char row, char col);
void   i2c_display_tset(int8_t tset);
void   i2c_init_display();
void   serial_ascii(int *fd, char val);
void   serial_curpos(int *fd, char row, char col);
void   serial_display_tset(int *fd_serial, int8_t tset);
void   serial_init_display(int *fd);
void   signal_callback_handler(int signum);

#define I2C_DELAY 10
#define GP_O_ON   25
#define GP_I_TDN  18
#define GP_I_TUP  27

int main() {

  signal(SIGINT,signal_callback_handler);

////// constants

  const int8_t tcycle   = 1; // mintes
  const int8_t tmeaspc  = 1;
  const int8_t set_bits = 6;
  const int8_t cyc_bits = 3;

////// variables

  int8_t   tset = 00 << 1; // 1/2 degrees
  struct timespec gettime_now;
  long int start_time, next_time, time_now, time_but;
  int8_t   count, state;
  float    tmeas, tacc;
  int16_t  terror, erracc;
  int8_t   pprop;
  int8_t   pinteg, power, hi_count;
  int8_t   intdis, intbit, propbit, mask, bit;
  int8_t   j;
  int8_t   init = 1;
    // 1-wire
  char     devPath[128];
    // serial
  int      fd_serial;
    // spi
//  char     spi_data[] = {0x00,0x00};
    // logfile
  struct tm *timeinfo;
  time_t   rawtime;
  char     log_basedir[] = "/home/pi/logs/";
  char     log_timestamp[128];
  char     log_filename[128];
  FILE     *logfile;

  int8_t   cycles = (1 << cyc_bits) - 1;
  float    tstate = (float)tcycle * 60.0 / (float)cycles;
  float    tcount = tstate / (float)tmeaspc;

  printf ("\ntcycle: %d tmeaspc: %d",tcycle,tmeaspc);
  printf (" tstate: %f tcount: %f\n",tstate,tcount);

////// setup I/O

  if ((fd_serial = serialOpen ("/dev/ttyAMA0", 9600)) < 0) {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
    }
  if (!setup_temp_sensor_1w(devPath))
    exit(1);
  if (!bcm2835_init())
    exit(1);
  setup_ports_bcm();
//  setup_spi_bcm();
//  setup_i2c_bcm();

  bcm2835_delay(500);
  bcm2835_delay(500);

////// initial outputs

  bcm2835_gpio_write(GP_O_ON,LOW);

//  i2c_init_display();
//  i2c_display_tset(tset);
  serial_init_display(&fd_serial);
  serial_display_tset(&fd_serial, tset);

////// logfile

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

  while (1) {

    if (init) {
      init       = 0;
      clock_gettime(CLOCK_REALTIME, &gettime_now);
        // times are scaled to units of 10mS
      start_time = (gettime_now.tv_sec*100) + (gettime_now.tv_nsec/10000000);
      count      = 0;
      state      = (2 * cycles) - 1;
      tacc       = 0;
      erracc     = 0;
      intdis     = 1;
      power      = 0;
      hi_count   = 0;
      }

    if (count != (tmeaspc - 1))
      next_time = start_time + (long int)(tcount * (float)(count + 1) * 100.0);
    else {
      next_time  = start_time + (long int)(tstate * 100.0);
      start_time = next_time;
      }

    if (!read_temp_1w(devPath, &tmeas))
      exit(1);
//    read_temp_i2c(&tmeas);
//    tmeas = 21.4;
//    printf("tmeas: %.3f\n",tmeas);
    tacc += tmeas;

////// state transition

    if (count == 0) {
      if (state < cycles)
        tacc /= (float)tmeaspc;

      terror = (tset << (5 + cyc_bits - set_bits))
               - (int16_t)(tacc * pow(2.0,cyc_bits));
      pprop  = fmax(-cycles,fmin(cycles,terror));

      erracc += terror;
      if (terror >= cycles)
        intdis = 1;

      state = ++state % cycles;

      if (state == 0) {
        hi_count = 0;
        /**/ if (intdis ==  1)        pinteg  = 0;
        else if (erracc >=  2*cycles) pinteg += (pinteg<(cycles-1));
        else if (erracc <= -2*cycles) pinteg -= (pinteg>0);;
        }

////// PWM

      power    = fmin(cycles,fmax(0,pinteg + pprop));
      intbit   = (pinteg > state);
      propbit  = (pprop >= (cycles - state));
      mask     = (power > hi_count);
      bit      = (intbit | propbit) & mask;
      hi_count += bit;

      bcm2835_gpio_write(GP_O_ON,(bit ? HIGH : LOW));

        //
      serial_curpos(&fd_serial,0x02,0x05);
      serial_ascii(&fd_serial,0x30 | (((uint8_t)tacc/10) & 0x0f));
      serial_ascii(&fd_serial,0x30 | (((uint8_t)tacc%10) & 0x0f));
      serial_ascii(&fd_serial,'.');
      serial_ascii(&fd_serial,0x30 | (((uint8_t)(tacc*10.0)%10) & 0x0f));
        //
      serial_curpos(&fd_serial,0x01,0x0a);
      for (j=7;j>0;j--)
        serial_ascii(&fd_serial,(power >= j) ? '<' : ' ');
        //
      serial_curpos(&fd_serial,0x02,0x10);
      serial_ascii(&fd_serial,0x30 | ((state+1) & 0x0f));

//      spi_data[0] = (tacc * pow(2.0,3));
//      spi_data[1] = power;
//      bcm2835_spi_transfern(spi_data,2);

      printf ("\n");
      printf ("state: %d", state);
      printf (" tset: %.1f", tset/2.0);
      printf (" tave: %.3f", tacc);
      printf (" tave_r: %.3f", (int)(tacc*pow(2.0,cyc_bits))/pow(2.0,cyc_bits));
      printf (" terror: %d", terror);
      printf ("\nerracc: %d", erracc);
      printf (" intdis: %d", intdis);
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
      while (!bcm2835_gpio_lev(GP_I_TDN)) {
        tset -= (tset>0);
        serial_display_tset(&fd_serial,tset);
        printf("%i\n",tset);
        if (!init) {
          bcm2835_delay(400);
          init = 1;
          }
        bcm2835_delay(600);
        } // while
      while (!bcm2835_gpio_lev(GP_I_TUP)) {
        tset += (tset<(30<<1));
        serial_display_tset(&fd_serial,tset);
        printf("%i\n",tset);
        if (!init) {
          bcm2835_delay(400);
          init = 1;
          }
        bcm2835_delay(600);
        }
      if (init) {
        bcm2835_delay(600);
        break;
        }

      clock_gettime(CLOCK_REALTIME, &gettime_now);
      time_now = (gettime_now.tv_sec*100) + (gettime_now.tv_nsec/10000000);
      if (time_now >= next_time)
        break;

      bcm2835_delay(100);
      } // end state transition

    } // end of while loop

  return EXIT_SUCCESS;

  } // end main

////////////////
////// functions
////////////////

void signal_callback_handler(int signum) {
  printf("\nCaught signal %d\n",signum);
    // Cleanup and close up stuff here
  bcm2835_gpio_write(GP_O_ON,LOW);
//  uint8_t spi_out[] = {0x01,0x01};
//  bcm2835_spi_transfern(spi_out,sizeof(spi_out));
//  bcm2835_spi_end();
//  bcm2835_i2c_end();
  bcm2835_close();
//  fclose(logfile);
    // Terminate program
  exit(signum);
  }

void serial_ascii(int *fd, char val) {
  serialPutchar(*fd,0x0a);
  serialPutchar(*fd,val);
  serialPutchar(*fd,0xff);
  }

void serial_curpos(int *fd, char row, char col) {
  serialPutchar(*fd,0x02);
  serialPutchar(*fd,row);
  serialPutchar(*fd,col);
  serialPutchar(*fd,0xff);
  }

void serial_display_tset(int *fd, int8_t tset) {
  serial_curpos(fd,0x01,0x05);
  serial_ascii(fd,0x30 | (tset/20 & 0x0f));
  serial_ascii(fd,0x30 | ((tset%20)/2 & 0x0f));
  serial_ascii(fd,'.');
  serial_ascii(fd,0x30 | ((tset*5)%10 & 0x0f));
  }

void serial_init_display(int *fd) {
    // set LCD type 16x2
  serialPutchar(*fd,0x05);
  serialPutchar(*fd,0x02);
  serialPutchar(*fd,0x10);
  serialPutchar(*fd,0xff);
    // clear display
  serialPutchar(*fd,0x04);
  serialPutchar(*fd,0xff);
    // Set
  serial_ascii(fd,'S');
  serial_ascii(fd,'e');
  serial_ascii(fd,'t');
    // line 2
  serialPutchar(*fd,0x03);
  serialPutchar(*fd,0x02);
  serialPutchar(*fd,0xff);
    // Temp
  serial_ascii(fd,'T');
  serial_ascii(fd,'m');
  serial_ascii(fd,'p');
    //
  serial_curpos(fd,0x02,0x0a);
  serial_ascii(fd,'^');
  serial_ascii(fd,' ');
  serial_ascii(fd,'C');
  serial_ascii(fd,'y');
  serial_ascii(fd,'c');
    // backlight 
  serialPutchar(*fd,0x07);
  serialPutchar(*fd,0x40);
  serialPutchar(*fd,0xff);
  }

int8_t i2c_ascii(char val) {
  char    i2c_data[] = {0x0a,val};
  uint8_t retval;

  bcm2835_i2c_setSlaveAddress(0x3A);
  retval = bcm2835_i2c_write(i2c_data,2);
  delay(I2C_DELAY);
  return(retval);
  }

int8_t i2c_curpos(char row, char col) {
  char    i2c_data[] = {0x02,row,col};
  uint8_t retval;

  bcm2835_i2c_setSlaveAddress(0x3A);
  retval = bcm2835_i2c_write(i2c_data,3);
  delay(I2C_DELAY);
  return(retval);
  }

void i2c_display_tset(int8_t tset) {
  i2c_curpos(0x01,0x05);
  i2c_ascii(0x30 | (tset/20 & 0x0f));
  i2c_ascii(0x30 | ((tset%20)/2 & 0x0f));
  i2c_ascii('.');
  i2c_ascii(0x30 | ((tset*5)%10 & 0x0f));
  }

void i2c_init_display() {
  char    i2c_data[3];

  bcm2835_i2c_setSlaveAddress(0x3A);
    // backlight 
  i2c_data[0] = 0x07;
  i2c_data[1] = 0x40;
  bcm2835_i2c_write(i2c_data,2);
  delay(I2C_DELAY);
    // set LCD type 16x2
  i2c_data[0] = 0x05;
  i2c_data[1] = 0x02;
  i2c_data[2] = 0x10;
  bcm2835_i2c_write(i2c_data,3);
  delay(I2C_DELAY);
    // clear display
  i2c_data[0] = 0x04;
  bcm2835_i2c_write(i2c_data,1);
  delay(I2C_DELAY);
    // Set
  i2c_ascii('S');
  i2c_ascii('e');
  i2c_ascii('t');
    // line 2
  i2c_data[0] = 0x03;
  i2c_data[1] = 0x02;
  bcm2835_i2c_write(i2c_data,2);
  delay(I2C_DELAY);
    // Temp
  i2c_ascii('T');
  i2c_ascii('m');
  i2c_ascii('p');
    //
  i2c_curpos(0x02,0x0a);
  i2c_ascii('^');
  i2c_ascii(' ');
  i2c_ascii('C');
  i2c_ascii('y');
  i2c_ascii('c');
  }

int8_t read_temp_i2c(float *tmeas) {
  char    i2c_data[2];
  uint8_t retval;

  bcm2835_i2c_setSlaveAddress(0x48); // TMP102
  i2c_data[0] = 0x00;
  bcm2835_i2c_write(i2c_data,1);
  delay(I2C_DELAY);
  retval = bcm2835_i2c_read(i2c_data,2);
  delay(I2C_DELAY);
  *tmeas = (float)((i2c_data[0]<<4) | (i2c_data[1]>>4)) / 16.0;
  return(retval);
  }

void setup_i2c_bcm() {
  bcm2835_i2c_begin();
//  bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); 
  }

void setup_spi_bcm() {
  bcm2835_spi_begin();
  bcm2835_gpio_fsel(RPI_GPIO_P1_26,BCM2835_GPIO_FSEL_OUTP);     // return CE1
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0,LOW);       // the default
  }

void setup_ports_bcm() {
    // outputs
  bcm2835_gpio_fsel(GP_O_ON,BCM2835_GPIO_FSEL_OUTP);
    // inputs
  bcm2835_gpio_fsel(GP_I_TDN,BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(GP_I_TDN,BCM2835_GPIO_PUD_UP);
  bcm2835_gpio_fsel(GP_I_TUP,BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(GP_I_TUP,BCM2835_GPIO_PUD_UP);
  }

int8_t read_temp_1w(char *devPath, float *tmeas) {
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

int8_t setup_temp_sensor_1w(char *devPath) {
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

