import serial
import time
import sys

ser = serial.Serial(
    port='/dev/tty.usbserial-A1014HTO',
    baudrate='9600',
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
#,    rtscts=0
)

#ser.open()
ser.isOpen()

print 'Enter command:\n'
#          MSB, LSB, AT,   FrameID, AT Cmd     Checksum
#a = bytearray(b'7E 00 04 08 01 49 44 72')
a = bytearray(b'0x7E 0x00 0x04 0x08 0x01 0x49 0x44 0x69')
#{0x7E, 0x00,   0x02,   0x08, 0x01,     ord('I'), ord('D'),  0xFF - ((ord('I') + ord('D')) & 0xFF)}

#ser.write(a)
ser.write(chr(0x7E))
ser.write(chr(0x00))
ser.write(chr(0x04))
ser.write(chr(0x08))
ser.write(chr(0x01))
ser.write(chr(0x49))
ser.write(chr(0x44))
ser.write(chr(0x69))

frameName = { 0x08:"AT Command",
              0x09:"AT Command - Queue Parameter Value",
              0x10:"ZigBee Transmit Request",
              0x11:"Explicit Addressing ZigBee Command Frame",
              0x17:"Remote Command Request",
              0x21:"Create Source Route",
              0x88:"AT Command Response",
              0x8A:"Modem Status",
              0x8B:"ZigBee Transmit Status",
              0x90:"ZigBee Receive Packet (AO=0)",
              0x91:"ZigBee Explicit Rx Indicator (AO=1)",
              0x92:"ZigBee IO Data Sample Rx Indicator",
              0x94:"XBee Sensor Read Indicator (AO=0)",
              0x95:"Node Identification Indicator (AO=0)",
              0x97:"Remote Command Response",
              0xA0:"Over-the-Air Firmware Update Status",
              0xA1:"Route Record Indicator",
              0xA3:"Many-to-One Route Request Indicator"}


class Frame:
    "A XBee ZB frame"
    def __init__(self, frameType, payload):
        self.frameType = frameType
        self.payload = payload
    
    def payload_str(self):
        ret = ""
        for (b) in self.payload:
            ret += " 0x%0.2X" % b
        return ret

    def dump(self):
        name = frameName[self.frameType]
        if (name == ''):
            return "Unknown frame type";
        else:
            return name + " (0x%0.2X" % self.frameType + ")" + self.payload_str()

class OpaqueFrame(Frame):
    "An unparsed XBee ZB frame"
    def __init__(self, frameType, payload):
        Frame.__init__(self, frameType, payload)

class IOSampleFrame(Frame):
    "An XBee ZB IO Sample frame"
    def __init__(self, frameType, payload):
        Frame.__init__(self, frameType, payload)

        self.sender_address  = (payload[0] << 7*8)
        self.sender_address |= (payload[1] << 6*8)
        self.sender_address |= (payload[2] << 5*8)
        self.sender_address |= (payload[3] << 4*8)
        self.sender_address |= (payload[4] << 3*8)
        self.sender_address |= (payload[5] << 2*8)
        self.sender_address |= (payload[6] << 1*8)
        self.sender_address |= (payload[7] << 0*8)

        self.source_network_address  = (payload[8] << 8)
        self.source_network_address |= (payload[9])

        self.receive_options = payload[10]

        self.number_of_samples = payload[11] # Always 1

        self.digital_sample_mask  = payload[12] << 8
        self.digital_sample_mask |= payload[13]

        self.analog_sample_mask = payload[14]

        if (self.digital_sample_mask > 0):
            self.digital_samples  = payload[15]
            self.digital_samples |= payload[16]
        else:
            self.digital_samples = 0
        
        # Assume no samples for now

    def payload_str(self):
        ret = "\n"
        ret += " sender_address(0x%0.16X" % self.sender_address + ")\n"
        ret += " source_network_address(0x%0.4X" % self.source_network_address + ")\n"
        ret += " receive_options(0x%0.2X" % self.receive_options + ")\n"
        ret += " number_of_samples(0x%0.2X" % self.number_of_samples + ")\n"
        ret += " digital_sample_mask(0x%0.4X" % self.digital_sample_mask + ")\n"
        ret += " analog_sample_mask(0x%0.2X" % self.analog_sample_mask + ")\n"
        ret += " digital_samples(0x%0.4X" % self.digital_samples + ")\n"
        return ret


class FrameFactory:
    @staticmethod
    def create(frameType, payload):
        if (frameType == 0x92):
            return IOSampleFrame(frameType, payload)
        else:
            return OpaqueFrame(frameType, payload)


class StateMachine:

    def __init__(self):
        self.state = "Receive"
        self.clear()

    def clear(self):
        self.length_msb_received = False
        self.length_lsb_received = False
        self.command_length = 0
        self.check_sum = 0
        self.payload = bytearray()
        self.frameType = 0
        
    def is_start_of_frame(self, byte):
        return byte == 0x7E

    def previous_frame_is_done(self):
        return self.state == "Receive"
        
    def start_frame(self):
        self.clear()
        self.state = "Read Length MSB"


    def expects_length_msb(self):
        return self.state == "Read Length MSB"

    def receive_length_msb(self, byte):
        self.command_length = byte << 8
        self.state = "Read Length LSB"


    def expects_length_lsb(self):
        return self.state == "Read Length LSB"

    def receive_length_lsb(self, byte):
        self.command_length |= byte
        self.state = "Read Payload"

        if (self.command_length == 0):
            sys.stderr.write("WARNING: Command of zero length. Dropping frame.")
            self.state = "Illegal"


    def expects_payload(self):
        return self.state == "Read Payload"

    def receive_payload(self, byte):
        if (self.frameType == 0):
            self.frameType = byte
            if (self.frameType == 0):
                sys.stderr.write("WARNING: Zero frame type. Dropping frame.")
                self.state = "Illegal"
        else:
            self.payload.append(byte)
        
        self.command_length -= 1
        self.check_sum += byte
        if (self.command_length == 0):
            self.state = "Read Checksum"

    def expects_checksum(self):
        return self.state == "Read Checksum"

    def read_and_validate_checksum(self, byte):
        if (0xFF - (self.check_sum & 0xFF) != byte):
            return False
        else:
            return True
        
    def finalize_frame(self):
        frame = FrameFactory.create(self.frameType, self.payload)
        self.state = "Receive"
        return frame


sm = StateMachine()
    
while 1 :
    while ser.inWaiting() > 0:
        byte = ord(ser.read(1))
        sys.stdout.write(hex(byte) + " ")

        # Start of frame
        if (sm.is_start_of_frame(byte)):
            if (not sm.previous_frame_is_done()):
                sys.stdout.write("--- Dropping previous command ---\n")
            sm.start_frame()

        elif (sm.expects_length_msb()):
            sm.receive_length_msb(byte)

        elif (sm.expects_length_lsb()):
            sm.receive_length_lsb(byte)

        elif (sm.expects_payload()):
            sm.receive_payload(byte)

        elif (sm.expects_checksum()):
            if (sm.read_and_validate_checksum(byte)):
                frame = sm.finalize_frame()
                sys.stdout.write(frame.dump() + "\n")
                sys.stdout.flush()
            else:
                sys.stdou.write("--- Checksum failed! ---\n");

        else:
            sys.stdout.write("--- Dropping garbage: " + str(byte) + "\n")

