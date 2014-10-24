import logging
import usb1

req_id = 64

POS_MIN = -100
POS_MAX = 100

logger = logging.getLogger( "sr.servo" )

class Servo(object):
    "A servo board"
    def __init__(self, path, busnum, devnum, serialnum = None):
        self.serialnum = serialnum

        self.ctx = usb1.USBContext()
        self.handle = None
        for dev in self.ctx.getDeviceList():
            if dev.getBusNumber() == busnum and dev.getDeviceAddress() == devnum:
                self.handle = dev.open()

        if self.handle is None:
            raise Exception("Failed to find servo board even though it was enumerated")


    def close(self):
        self.handle.close()

    def __setitem__(self, index, value):
        if index < 0 or index > 12:
            return
        # Limit the value to within the valid range
        if value > POS_MAX:
            value = POS_MAX
        elif value < POS_MIN:
            value = POS_MIN
        self.handle.controlWrite(0, req_id, value, index, "")

    def __repr__(self):
        return "Servo( serialnum = \"{0}\" )".format( self.serialnum )