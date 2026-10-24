#!/usr/bin/env python3
"""DDSM115 protocol 0x74 only: no setpoint, mode change or ID broadcast."""
import json
import time
import serial

def crc(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8c if value & 1 else 0)
    return value

with serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABADW8KY-if00-port0',
                   115200, timeout=.15, exclusive=True) as port:
    for motor in (4, 3):
        for sample in range(3):
            port.reset_input_buffer()
            frame = bytes([motor, 0x74, 0, 0, 0, 0, 0, 0, 0])
            port.write(frame + bytes([crc(frame)]))
            port.flush()
            reply = port.read(10)
            result = {'motor': motor, 'sample': sample, 'raw': reply.hex()}
            if len(reply) == 10 and reply[0] == motor and crc(reply[:9]) == reply[9]:
                result.update(mode=reply[1],
                    current_A=int.from_bytes(reply[2:4], 'big', signed=True)*8/32767,
                    rpm=int.from_bytes(reply[4:6], 'big', signed=True),
                    temperature_C=reply[6], fault=reply[8])
            else:
                result['error'] = 'invalid reply'
            print(json.dumps(result))
            time.sleep(.1)
