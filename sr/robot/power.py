import collections

CMD_ENABLE_INPUT_NOTES = 5
CMD_PLAY_PIEZO = 6
CMD_SET_LEDS = 7
CMD_SET_MOTOR_RAIL = 8
CMD_GET_LEDS = 9
CMD_GET_VI = 10
CMD_GET_STACK = 11

class LedList(object):
    def __init__(self, dev=None):
        self.dev = dev

    def __len__(self):
        return 3

    def __setitem__(self, idx, val):
        if idx > 2 or idx < 0:
            raise IndexError("The powerboard only has 3 LEDs")

        with self.dev.lock:
            # Fetch current status of led
            r = self._get_leds_nolock()
            bit = bool( r & (1 << idx) )

            # Normalise val
            val = bool(val)

            if (bit != val):
                flags = r & (~(1 << idx))
                if val:
                    flags |= (1 << idx)
                tx = [ CMD_SET_LEDS, flags ]
                self.dev.txrx( tx )

    def __getitem__(self, idx):
        if idx > 2 or idx < 0:
            raise IndexError("The powerboard only has 3 LEDs")

        r = self._get_leds()
        return bool( r & (1 << idx) )

    def _get_leds(self):
        """Read the state of all the LEDs.
        Return the values in a bitmask."""
        with self.dev.lock:
            return self._get_leds_nolock()

    def _get_leds_nolock(self):
        """Read the state of all the LEDs
        But don't acquire the device lock."""

        tx = [ CMD_GET_LEDS ]
        rx = self.dev.txrx( tx )
        return rx[0]

class Battery(object):
    def __init__(self, dev=None):
        self.dev = dev

    @property
    def voltage(self):
        return round(self._get_vi()[0], 2)

    @property
    def current(self):
        return round(self._get_vi()[1], 2)

    def _get_vi(self):
        """Read the battery voltage and current from the power board.
	Return the values in a tuple."""
        with self.dev.lock:
            r = self.dev.txrx( [ CMD_GET_VI ] )

	# Use scaling values stated in monitor.h of power-fw.git
        v = (r[0] | (r[1] << 8)) * 0.0036621
        i = (r[2] | (r[3] << 8)) * 0.012201
        return v, i

# Stack Usage namedtuple
# Fields:
#  - allocated: The maximum amount of RAM that can safely be used by the MSP
#  - peak_use: The peak amount of stack used by the MSP
StackUsage = collections.namedtuple("StackUsage", ["allocated", "peak_use"])

class Power:
    def __init__(self, dev):
        self.dev = dev
        self.led = LedList(dev)
        self.battery = Battery(dev)
        self._set_motor_rail(True)

    def beep( self, freq = 1000, dur = 0.1 ):
        "Beep"

        if hasattr( freq, "__iter__" ):
            beeps = freq
        else:
            "It's just a single note"
            beeps = [(freq, dur)]

        MAX_BEEPS = 10
        if len(beeps) > MAX_BEEPS:
            # TODO: Do something better here
            raise Exception("Can not queue more than {0} beeps at a time.".format(MAX_BEEPS))

        tx = [ CMD_PLAY_PIEZO, len(beeps) ]
        for f,d  in beeps:
            d = int(1000 * d)

            # Frequency
            tx.append( (f >> 8) & 0xff )
            tx.append( f & 0xff )
            # Duration
            tx.append( (d >> 8) & 0xff )
            tx.append( d & 0xff )

            # Volume (fixed right now)
            tx.append( 5 )

        with self.dev.lock:
            self.dev.txrx( tx )

    def _set_motor_rail(self, en):
        """Enable/disable the motor rail on the power board"""
        tx = [CMD_SET_MOTOR_RAIL, bool(en)]
        with self.dev.lock:
            self.dev.txrx(tx)

    def _get_stack_usage(self):
        """Return the stack space and max used stack space"""
        tx = [CMD_GET_STACK]
        with self.dev.lock:
            r = self.dev.txrx(tx)
        return StackUsage( r[0] | (r[1]<<8), r[2] | (r[3]<<8) )
