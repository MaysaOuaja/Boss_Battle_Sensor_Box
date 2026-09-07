"""Start the walkie-talkie only when its button is pressed during boot."""

import machine
import time
import gc


def _bmi_write(i2c, register, value):
    i2c.writeto_mem(0x68, register, bytes([value]))
    time.sleep_ms(1)


def _configure_bmi():
    i2c = machine.I2C(0, scl=machine.Pin("P8_0"), sda=machine.Pin("P8_1"), freq=400000)
    if i2c.readfrom_mem(0x68, 0, 1)[0] != 0x24:
        raise OSError("BMI270 nicht gefunden")
    _bmi_write(i2c, 0x7E, 0xB6)
    time.sleep_ms(250)
    for register in (0x7C, 0x59, 0x5B, 0x5C):
        _bmi_write(i2c, register, 0)
    buffer = bytearray(16)
    config = open("/bmi270_config.bin", "rb")
    try:
        for index in range(512):
            if config.readinto(buffer) != 16:
                raise OSError("BMI270-Konfiguration unvollstaendig")
            i2c.writeto_mem(0x68, 0x5E, buffer)
            time.sleep_ms(1)
            if index < 511:
                position = (index + 1) * 8
                _bmi_write(i2c, 0x5B, position & 15)
                _bmi_write(i2c, 0x5C, position >> 4)
    finally:
        config.close()
    _bmi_write(i2c, 0x59, 1)
    for _ in range(10):
        if i2c.readfrom_mem(0x68, 0x21, 1)[0] & 1:
            break
        time.sleep_ms(100)
    else:
        raise OSError("BMI270-Initialisierung fehlgeschlagen")
    for register, value in ((0x7E, 0xB0), (0x7D, 0x0E), (0x40, 0x0A), (0x42, 0x0A), (0x7C, 2), (0x41, 1), (0x43, 0)):
        _bmi_write(i2c, register, value)
    return i2c


button = machine.Pin("P17_1", machine.Pin.IN, machine.Pin.PULL_UP)
print("Walkie-talkie: Taste innerhalb von 4 Sekunden druecken zum Starten.")
started = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), started) < 4000:
    if button.value() == 0:
        print("Starttaste erkannt. Walkie-talkie startet.")
        i2c = _configure_bmi()
        gc.collect()
        import device_main
        device_main.main(bmi_configured=True, i2c=i2c)
        break
    time.sleep_ms(20)
else:
    print("Walkie-talkie bereit.")