# hello

import RPi.GPIO as GPIO
import os
import glob
import time

# initialise 1-wire sensor

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

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

# GPIO event triggers

def display_temp(temp):
    GPIO.output(23,((temp & 16) != 0))
    GPIO.output(22,((temp &  8) != 0))
    GPIO.output(27,((temp &  4) != 0))
    GPIO.output(18,((temp &  2) != 0))
    GPIO.output(17,((temp &  1) != 0))

def cb_tset(channel):  
    global tset, init
    init = 1
    if (channel == 25):
      tset -= 1
    elif (channel == 24):
      tset += 1
    display_temp(tset)

GPIO.add_event_detect(25, GPIO.FALLING, callback = cb_tset, bouncetime = 250)
GPIO.add_event_detect(24, GPIO.FALLING, callback = cb_tset, bouncetime = 250)

# initialise 1-wire sensor

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

# open logfile

log_base_dir  = '/home/pi/logs/'
log_timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
log_filename  = "".join([log_base_dir, "tlog_", log_timestamp, ".csv"])
log_datafile  = open(log_filename, "w", 1)

log_datafile.write("date,time,temp,state,prop,int,power,carry,on\n")

# constants

tset     = 18 # degrees
tcycle   = 10 # mintes

set_bits = 5
cyc_bits = 3
tmeaspc  = 5

cycles   = (1 << cyc_bits) - 1
tsleep   = float(tcycle) * 60.0 / float(cycles * tmeaspc)

display_temp(tset)

print ""
print "tcycle:", tcycle, " cycles:", cycles, " tsleep:", tsleep

# variables

init = 1

try:
    while (1):
        time_1 = time.time()

        if (init):
            count    = 0
            state    = (2 * cycles) - 1
            tacc     = 0
            erracc   = 0
            intdis   = 1
            power    = 0
            hi_count = 0
            init     = 0

#         tmeas = 20.5
        tmeas = read_temp()
        tacc  = tacc + tmeas
#         print ""
#         print "tmeas:", tmeas,
#         print " tacc:", tacc

        if (count == 0):
            if (state < cycles):
                tave = tacc / float(tmeaspc)
            else:             # single temp measurement at init
                tave = tacc
            tacc = 0

            state = (state + 1) % cycles

            terror = (tset << (5 + cyc_bits - set_bits)) \
                     - int(tave * 2.0**cyc_bits)

# integral path

            erracc = erracc + terror

            if (terror > cycles):
                intdis = 1

            if (state == 0):
                if (intdis == 1):
                    pinteg = 0
                elif (erracc >=  5*cycles): pinteg += 2
                elif (erracc >=  2*cycles): pinteg += 1
                elif (erracc <= -5*cycles): pinteg -= 2
                elif (erracc <= -2*cycles): pinteg -= 1
                pinteg = min(cycles-1,max(0,pinteg));

# proportional path

            pprop = min(cycles,terror)

            if (state == 0):
                carry    = min(2,max(-1,power - hi_count))
                hi_count = 0

# PWM

            power    = carry + pinteg + pprop
            intbit   = ((carry + pinteg) > state)
            propbit  = (pprop >= (cycles - state))
            mask     = (power > hi_count)
            bit      = (intbit | propbit) & mask
            hi_count = hi_count + bit

            log_datafile.write(time.strftime('%Y-%m-%d, %H:%M:%S'))
            log_datafile.write(", " + str(int(tave * 16.0)/16.0))
            log_datafile.write(", " + str(state))
            log_datafile.write(", " + str(pprop))
            log_datafile.write(", " + str(pinteg))
            log_datafile.write(", " + str(power))
            log_datafile.write(", " + str(carry))
            log_datafile.write(", " + str(bit*1))
            log_datafile.write("\n")

            print ""
            print time.strftime('%Y-%m-%d %H:%M:%S')
            print "state:",   state,
            print " tset:",   tset,
            print " tave_f:", tave,
            print " tave_i:", float(int(tave * 16.0)) / 16.0,
            print " terror:", terror
            print "erracc:",  erracc,
            print " intdis:", intdis,
            print " carry:",  carry,
            print " pinteg:", pinteg,
            print " pprop:",  pprop,
            print " power:",  power,
            print " bit:",    bit*1,
            print " hi_c:",   hi_count

            GPIO.output(7,(bit & mask))

            GPIO.output(15,((power & 8) != 0))
            GPIO.output(14,((power & 4) != 0))
            GPIO.output(11,((power & 2) != 0))
            GPIO.output(10,((power & 1) != 0))
            GPIO.output(9,(state == 0))

            if (state == 0):
                erracc = 0
                intdis = 0

# end of new state section

        count = (count + 1) % tmeaspc

        while ((time.time() - time_1) < tsleep):
            time.sleep(0.2)
            if (init):
                break

except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt
    GPIO.cleanup()        # resets all GPIO ports used by this program
    log_datafile.close()  # close logfile

