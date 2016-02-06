import smbus

bus = smbus.SMBus(1)

bus.write_byte(0x27, 0b10000000)
