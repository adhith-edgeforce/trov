import smbus2, time

BUS, ADDR = 7, 0x48
bus = smbus2.SMBus(BUS)

while True:
    bus.write_i2c_block_data(ADDR, 0x01, [0xC2, 0x83])
    time.sleep(0.1)
    data = bus.read_i2c_block_data(ADDR, 0x00, 2)
    raw = (data[0] << 8) | data[1]
    if raw > 32767:
        raw -= 65536
    voltage = raw * 4.096 / 32768.0
    print(f"Voltage: {voltage:.4f} V", end='\r')
