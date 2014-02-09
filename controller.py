# hello

import RPi.GPIO as GPIO
import os
import glob
import time

# initialise ports

GPIO.setmode(GPIO.BCM)

GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(14, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)
GPIO.setup( 9, GPIO.OUT)
# GPIO.setup( 8, GPIO.OUT)
GPIO.setup( 7, GPIO.OUT)
# GPIO.setup( 4, GPIO.OUT) # 1-wire temperature sensor
# GPIO.setup( 3, GPIO.OUT)
# GPIO.setup( 2, GPIO.OUT)

GPIO.output(7,0)

# initialise 1-wire sensor

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

w1_base_dir = '/sys/bus/w1/devices/'
w1_device_folder = glob.glob(w1_base_dir + '28*')[0]
w1_device_file = w1_device_folder + '/w1_slave'

def read_temp_raw():
    f = open(w1_device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c

def display_temp(temp):
    GPIO.output(23,((temp & 16) != 0))
    GPIO.output(22,((temp &  8) != 0))
    GPIO.output(27,((temp &  4) != 0))
    GPIO.output(18,((temp &  2) != 0))
    GPIO.output(17,((temp &  1) != 0))

def cb_tset(channel):  
    global tset
    if (channel == 25):
      tset -= 1
    elif (channel == 24):
      tset += 1
    display_temp(tset)

# GPIO event triggers

GPIO.add_event_detect(25, GPIO.FALLING, callback = cb_tset, bouncetime = 250)
GPIO.add_event_detect(24, GPIO.FALLING, callback = cb_tset, bouncetime = 250)

# open logfile

log_base_dir  = '/home/pi/logs/'
log_timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
log_filename  = "".join([log_base_dir, "tlog_", log_timestamp, ".log"])
log_datafile  = open(log_filename, "w", 1)

# constants

tset     = 18
set_bits = 5

cyc_bits = 3
tcycle   = 10 # mintes
tmeaspc  = 5

cycles   = (1 << cyc_bits) - 1
tsleep   = float(tcycle) * 60.0 / float(cycles * tmeaspc)

print ""
print "tcycle:", tcycle, " cycles:", cycles, " tsleep:", tsleep

# variables

count      = tmeaspc - 1
state      = (2 * cycles) - 1
tacc       = 0
erracc     = 0
intdis     = 1

display_temp(tset)

try:
    while (1):
#         tmeas = 20.5
        tmeas = read_temp()
        tacc  = tacc + tmeas
#         print ""
#         print "tmeas:", tmeas,
#         print " tacc:", tacc

        count = (count + 1) % tmeaspc

        if (count == 0):
            if (state >= cycles):
                tave = tacc
            else:
                tave = tacc / float(tmeaspc)

            state = (state + 1) % cycles

            terror = (tset << (5 + cyc_bits - set_bits)) \
                     - int(tave * float(1 << cyc_bits))
            tacc   = 0

            pprop  = min(cycles,terror)
            erracc = erracc + terror

            print ""
            print time.strftime('%Y-%m-%d %H:%M:%S')
            print "state:",   state,
            print " tset:",   tset,
            print " tave_f:", tave,
            print " tave_i:", float(int(tave * 16.0)) / 16.0,
            print " terror:", terror
            print "pprop:", pprop,
            print " erracc:", erracc,

            if (intdis == 0) & (pprop == cycles):
                intdis = 1

            if (state == 0):
                if (intdis == 1):
                    pinteg = 0
                elif (erracc >=   5*cycles): pinteg += 2
                elif (erracc >=   2*cycles): pinteg += 1
                elif (erracc <=  -5*cycles): pinteg -= 2
                elif (erracc <=  -2*cycles): pinteg -= 1
                pinteg = min(cycles-1,max(0,pinteg));
                erracc = 0
                intdis = 0

                hi_count = 0

            power    = min(cycles,max(0,pprop + pinteg))
            bit      = (pinteg > state) | (pprop >= cycles - state);
            mask     = (power > hi_count);
            hi_count = hi_count + (bit & mask);

            log_datafile.write(time.strftime('%Y-%m-%d %H-%M-%S'))
            log_datafile.write(" " + str(tave))
            log_datafile.write(" " + str(terror))
            log_datafile.write(" " + str((bit & mask)*1))
            log_datafile.write("\n")

            print " intdis:", intdis,
            print " pinteg:", pinteg,
            print " power:", power,
            print " out:", (bit & mask)*1,
            print " hi_c:", hi_count

            GPIO.output(7,(bit & mask))

            GPIO.output(15,((power & 8) != 0))
            GPIO.output(14,((power & 4) != 0))
            GPIO.output(11,((power & 2) != 0))
            GPIO.output(10,((power & 1) != 0))
            GPIO.output(9,(state == 0))

        time.sleep(tsleep)

except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt
    GPIO.cleanup()        # resets all GPIO ports used by this program
    log_datafile.close()

