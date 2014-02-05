# hello

import time
import RPi.GPIO as GPIO

# initialise ports

GPIO.setmode(GPIO.BCM)

GPIO.setup(25, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
# GPIO.setup(27, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(14, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
# GPIO.setup(10, GPIO.OUT)
# GPIO.setup( 9, GPIO.OUT)
# GPIO.setup( 8, GPIO.OUT)
# GPIO.setup( 7, GPIO.OUT)
GPIO.setup( 4, GPIO.OUT)
# GPIO.setup( 3, GPIO.OUT)
# GPIO.setup( 2, GPIO.OUT)

# constants

adc_bits = 9
set_bits = 6
pwm_div  = 2

# inputs

tset    = 40
adc_val = 280

# variables

count = 0
state = 14
last_state = 0
pinteg = 0
pprop = 0
power = 0
terror = 100
hi_count = 0
bit = 0
mask = 0
intdis = 0
erracc = 0

while (terror != 0):
    last_state = state
    state      = count >> pwm_div

    if (state != last_state):
        terror = (tset << (adc_bits - set_bits)) - adc_val

        if (state == 0):
            if (intdis == 1):
                pinteg = 0
            elif (erracc >=  11*15): pinteg = pinteg + 3
            elif (erracc >=   7*15): pinteg = pinteg + 2
            elif (erracc >=   2*15): pinteg = pinteg + 1
            elif (erracc <=  -2*15): pinteg = pinteg - 1
            elif (erracc <=  -7*15): pinteg = pinteg - 2
            elif (erracc <= -11*15): pinteg = pinteg - 3
            pinteg = min(15,max(0,pinteg));

            intdis   = 0
            erracc   = 0
            hi_count = 0

        pprop = min(15,terror)

        if (intdis == 0):
            if (pprop == 15):
                intdis = 1
            else:
                erracc = erracc + terror

        power    = pprop + pinteg
        bit      = (pinteg > state) | (pprop >= 15-state);
        mask     = (power >  hi_count);
        hi_count = hi_count + (bit & mask);

        GPIO.output(25, ((power & 8) != 0))
        GPIO.output(24, ((power & 4) != 0))
        GPIO.output(23, ((power & 2) != 0))
        GPIO.output(22, ((power & 1) != 0))

        GPIO.output(18, ((hi_count & 8) != 0))
        GPIO.output(17, ((hi_count & 4) != 0))
        GPIO.output(15, ((hi_count & 2) != 0))
        GPIO.output(14, ((hi_count & 1) != 0))

        GPIO.output(11, (state == 0))
        GPIO.output( 4, (bit & mask))

        print "state:", state,
        print "power:", power, "pprop:", pprop, "pinteg:", pinteg,
        print "bit:", bit, "mask:", mask, "out:", (bit & mask), "hi_count:", hi_count,

        adc_val = adc_val + (bit & mask)
#        print "adc_val:", adc_val
        print ""

    if (count < (15<<pwm_div)-1): count = count + 1
    else: count = 0

    time.sleep(0.2)

