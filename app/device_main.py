"""Four live sensor modes for the assembled walkie-talkie board."""

import gc
import machine
import network
import time

try:
    from survival_config import LED_BRIGHTNESS, TARGET_WINDOW, TEMPERATURE_OFFSET_C, WIFI_TARGET_SSID
except ImportError:
    WIFI_TARGET_SSID = "Eric"
    TEMPERATURE_OFFSET_C = -13.5
    LED_BRIGHTNESS = 0.2
    TARGET_WINDOW = 4

BUTTON_PIN = "P17_1"
LED_PIN = "P10_5"
NEOPIXEL_PIN = "P17_0"
SHT40_ADDRESS = 0x44
BMI_ADDRESS = 0x68
BUTTON_DEBOUNCE_MS = 250
WIFI_SCAN_INTERVAL_MS = 5000
UPDATE_INTERVAL_MS = 1000
WIFI_WATCHDOG_MS = 12000
GYRO_AXIS = 2
GYRO_CALIBRATION_SAMPLES = 120
GYRO_CALIBRATION_SETTLE_MS = 500
GYRO_CALIBRATION_MAX_SPREAD = 5.0
BUTTON_LONG_PRESS_MS = 1500
TURN_TIMEOUT_MS = 20000
WIFI_LEDS = (7, 8, 9)
DIRECTION_LEDS = (10, 11, 12)
ENVIRONMENT_LEDS = (4, 5, 6)
MOTION_LEDS = (1, 2, 3)
DOA_CM33_CLIENT_ID = 3
DOA_CM55_CLIENT_ID = 5
DOA_HEARTBEAT_CMD = 0x40
DOA_RESULT_CMD = 0x41
DOA_DIAGNOSTIC_CMD = 0x42
DOA_PDM_STATUS_CMD = 0x43
DOA_LABELS = ("unlabeled", "Back_Left", "Back", "Back_Right", "Front_Left", "Front", "Front_Right", "Left", "Noise", "Right")
_watchdog = None
_doa_state = [-1, 0]
_doa_ipc = None
_doa_last_report_ms = 0
_dps = None


def _dps_signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


class _Dps3xx:
    def __init__(self, i2c, addr=0x77):
        self.bus = i2c
        self.addr = addr
        self.kp = 1040384.0
        self.kt = 1040384.0
        for register, value in ((0x0E, 0xA5), (0x0F, 0x96), (0x62, 2), (0x0E, 0), (0x0F, 0),
                                (0x06, 0x26), (0x07, 0xA6), (0x08, 7), (0x09, 0x0C)):
            self.bus.writeto(self.addr, bytes((register, value)))

    def _raw(self, register):
        data = self.bus.readfrom_mem(self.addr, register, 3)
        return _dps_signed((data[0] << 16) | (data[1] << 8) | data[2], 24)

    def _pressure_coefficients(self):
        data = self.bus.readfrom_mem(self.addr, 0x13, 15)
        c00 = _dps_signed((data[0] << 12) | (data[1] << 4) | (data[2] >> 4), 20)
        c10 = _dps_signed(((data[2] & 15) << 16) | (data[3] << 8) | data[4], 20)
        c01 = _dps_signed((data[5] << 8) | data[6], 16)
        c11 = _dps_signed((data[7] << 8) | data[8], 16)
        c21 = _dps_signed((data[11] << 8) | data[12], 16)
        c20 = _dps_signed((data[9] << 8) | data[10], 16)
        c30 = _dps_signed((data[13] << 8) | data[14], 16)
        return c00, c10, c20, c30, c01, c11, c21

    def measurePressureOnce(self):
        pressure = self._raw(0x00) / self.kp
        temperature = self._raw(0x03) / self.kt
        c00, c10, c20, c30, c01, c11, c21 = self._pressure_coefficients()
        return c00 + pressure * (c10 + pressure * (c20 + pressure * c30)) + temperature * (c01 + pressure * (c11 + pressure * c21))


class _Pixels:
    def __init__(self, pin):
        self.pin = pin
        self.pixels = self
        self.buffer = bytearray(39)
        self.pin.init(machine.Pin.OUT)
        self.off()

    def __setitem__(self, index, color):
        offset = index * 3
        self.buffer[offset] = int(color[1] * LED_BRIGHTNESS)
        self.buffer[offset + 1] = int(color[0] * LED_BRIGHTNESS)
        self.buffer[offset + 2] = int(color[2] * LED_BRIGHTNESS)

    def write(self):
        machine.bitstream(self.pin, 0, (400, 850, 800, 450), self.buffer)

    def off(self):
        for index in range(39):
            self.buffer[index] = 0
        self.write()


def _bmi_write(i2c, register, value):
    i2c.writeto_mem(BMI_ADDRESS, register, bytes([value]))
    time.sleep_ms(1)


def _configure_bmi(i2c):
    if i2c.readfrom_mem(BMI_ADDRESS, 0, 1)[0] != 0x24:
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
            i2c.writeto_mem(BMI_ADDRESS, 0x5E, buffer)
            time.sleep_ms(1)
            if index < 511:
                position = (index + 1) * 8
                _bmi_write(i2c, 0x5B, position & 15)
                _bmi_write(i2c, 0x5C, position >> 4)
    finally:
        config.close()
    _bmi_write(i2c, 0x59, 1)
    for _ in range(10):
        if i2c.readfrom_mem(BMI_ADDRESS, 0x21, 1)[0] & 1:
            break
        time.sleep_ms(100)
    else:
        raise OSError("BMI270-Initialisierung fehlgeschlagen")
    for register, value in ((0x7E, 0xB0), (0x7D, 0x0E), (0x40, 0x0A), (0x42, 0x0A), (0x7C, 2), (0x41, 1), (0x43, 0)):
        _bmi_write(i2c, register, value)


def _bmi_values(i2c, register, scale):
    data = i2c.readfrom_mem(BMI_ADDRESS, register, 6)
    result = []
    for index in range(0, 6, 2):
        value = data[index] | (data[index + 1] << 8)
        result.append((value - 65536 if value >= 32768 else value) / scale)
    return result


def _crc8(data):
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x31) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def _read_sht40(i2c):
    i2c.writeto(SHT40_ADDRESS, b"\xfd")
    time.sleep_ms(10)
    data = i2c.readfrom(SHT40_ADDRESS, 6)
    if _crc8(data[:2]) != data[2] or _crc8(data[3:5]) != data[5]:
        raise OSError("SHT40 CRC-Pruefung fehlgeschlagen")
    temperature = -45 + 175 * ((data[0] << 8) | data[1]) / 65535
    humidity = max(0, min(100, -6 + 125 * ((data[3] << 8) | data[4]) / 65535))
    return temperature, humidity


def _clear(pixels):
    for index in range(13):
        pixels.pixels[index] = (0, 0, 0)


def _color(score):
    score = max(0.0, min(1.0, score))
    return (255, int(510 * score), 0) if score < 0.5 else (int(510 * (1.0 - score)), 255, 0)


def _meter(pixels, indexes, score):
    _clear(pixels)
    if score > 0:
        color = _color(score)
        for index in indexes[:min(len(indexes), max(1, int(score * len(indexes) + 0.999)))]:
            pixels.pixels[index] = color
    pixels.pixels.write()


def _feed_watchdog():
    if _watchdog is not None:
        _watchdog.feed()


def _doa_callback(client):
    global _doa_last_report_ms
    if client.cmd == DOA_HEARTBEAT_CMD:
        messages = {
            0x444F4131: "Audio-DoA auf CM55 bereit.",
            0x444F4132: "Audio-DoA: PDM-Mikrofone bereit.",
            0x444F4133: "Audio-DoA: Deepcraft-Initialisierung startet.",
            0x444F4134: "Audio-DoA: Deepcraft bereit.",
            0x444F4135: "Audio-DoA: PDM-Initialisierung startet.",
            0x444F4136: "Audio-DoA: PDM bereit.",
            0x444F4137: "Audio-DoA: Rechtes Mikrofon bereit.",
            0x444F4146: "Audio-DoA: PDM-Initialisierung fehlgeschlagen.",
            0x444F4140: "Audio-DoA: Audiofenster bereit.",
            0x444F4141: "Audio-DoA: Modellbeschreibung wird geladen.",
            0x444F4142: "Audio-DoA: Modellbeschreibung bereit.",
            0x444F4143: "Audio-DoA: U55-NPU wird initialisiert.",
            0x444F4144: "Audio-DoA: U55-NPU bereit.",
            0x444F4150: "Audio-DoA Speicherprobe: vor CM55-DTCM-Zugriff.",
            0x444F4154: "Audio-DoA Speicherprobe: CM55-DTCM bereit.",
            0x444F4151: "Audio-DoA Speicherprobe: Fenster 1 bereit.",
            0x444F4152: "Audio-DoA Speicherprobe: Fenster 2 bereit.",
            0x444F4153: "Audio-DoA Speicherprobe: Arena bereit.",
        }
        print(messages.get(client.value, "Audio-DoA Init-Fehler: 0x{:08x}".format(client.value)))
    elif client.cmd == DOA_DIAGNOSTIC_CMD:
        frames = (client.value >> 16) & 0xFFFF
        results = client.value & 0xFFFF
        print("Audio-DoA Diagnose: {} Audioframes | {} Modellresultate".format(frames, results))
    elif client.cmd == DOA_PDM_STATUS_CMD:
        right = (client.value >> 16) & 0xFFFF
        left = client.value & 0xFFFF
        print("Audio-DoA PDM: links {} | rechts {} Interrupts".format(left, right))
    elif client.cmd == DOA_RESULT_CMD:
        label = client.value & 0xFFFF
        confidence = (client.value >> 16) & 0xFFFF
        _doa_state[0] = label
        _doa_state[1] = confidence
        now = time.ticks_ms()
        if 0 <= label < len(DOA_LABELS) and time.ticks_diff(now, _doa_last_report_ms) >= 500:
            print("Audio-DoA: {} | Confidence {:.1f}%".format(DOA_LABELS[label], confidence / 10.0))
            _doa_last_report_ms = now


def _start_doa():
    global _doa_ipc
    _doa_ipc = machine.IPC(src_core=machine.IPC.CM33, target_core=machine.IPC.CM55)
    _doa_ipc.init()
    if not _doa_ipc.register_client(DOA_CM33_CLIENT_ID, _doa_callback, 1, 1):
        raise OSError("Audio-DoA IPC-Client konnte nicht registriert werden")
    _doa_ipc.enable_core(machine.IPC.CM55)
    time.sleep_ms(100)
    _doa_ipc.send(machine.IPC.CMD_START, 0, DOA_CM55_CLIENT_ID)


def _pressed(button, previous, last_press):
    current = button.value()
    now = time.ticks_ms()
    return current == 0 and previous and time.ticks_diff(now, last_press) >= BUTTON_DEBOUNCE_MS, current, now


def _target_network(wlan, bssid=None):
    _feed_watchdog()
    networks = wlan.scan(ssid=WIFI_TARGET_SSID)
    _feed_watchdog()
    if not networks:
        return None
    if bssid is None:
        return max(networks, key=lambda network: network[3])
    for network in networks:
        if network[1] == bssid:
            return network
    return None


def _name(network):
    try:
        return network[0].decode("utf-8") or "<versteckt>"
    except UnicodeError:
        return "<ungueltige SSID>"


def _scan_wifi_networks(wlan):
    _feed_watchdog()
    networks = wlan.scan()
    _feed_watchdog()
    if not networks:
        print("WLAN-Suche: Keine Netzwerke gefunden.")
        return
    print("WLAN-Suche: {} Netzwerke gefunden.".format(len(networks)))
    for network in networks:
        print("{} | Kanal {} | {} dBm".format(_name(network), network[2], network[3]))


def _wifi_strength(i2c, pixels, wlan, button):
    print("WLAN-Praesenz: Eric suchen und Signalstaerke anzeigen. Kurz: naechster Modus. Lang: WLAN-Peilung.")
    _scan_wifi_networks(wlan)
    previous = button.value()
    last_press = time.ticks_ms()
    while True:
        network = _target_network(wlan)
        if network is None:
            _meter(pixels, WIFI_LEDS, 0)
            print("WLAN-Praesenz: {} nicht gefunden.".format(WIFI_TARGET_SSID))
        else:
            _meter(pixels, WIFI_LEDS, (network[3] + 90) / 50)
            print("WLAN-Praesenz: {} | {} dBm".format(_name(network), network[3]))
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < WIFI_SCAN_INTERVAL_MS:
            _feed_watchdog()
            current = button.value()
            now = time.ticks_ms()
            if current == 0:
                if previous:
                    last_press = now
                elif time.ticks_diff(now, last_press) >= BUTTON_LONG_PRESS_MS:
                    _wifi_direction(i2c, pixels, wlan, button)
                    return time.ticks_ms()
            elif not previous and time.ticks_diff(now, last_press) < BUTTON_LONG_PRESS_MS:
                return now
            previous = current
            time.sleep_ms(20)


def _calibrate_gyro(i2c):
    total = 0.0
    lowest = None
    highest = None
    time.sleep_ms(GYRO_CALIBRATION_SETTLE_MS)
    for _ in range(GYRO_CALIBRATION_SAMPLES):
        _feed_watchdog()
        value = _bmi_values(i2c, 0x12, 16.384)[GYRO_AXIS]
        total += value
        lowest = value if lowest is None or value < lowest else lowest
        highest = value if highest is None or value > highest else highest
        time.sleep_ms(10)
    return total / GYRO_CALIBRATION_SAMPLES, highest - lowest


def _heading(i2c, state):
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, state[1])
    if elapsed >= 10:
        state[1] = now
        if elapsed <= 100:
            state[0] += (_bmi_values(i2c, 0x12, 16.384)[GYRO_AXIS] - state[2]) * elapsed / 1000.0
    return state[0]


def _turn(i2c, state, target, pixels, indicator, button):
    blink_on = False
    last_blink = time.ticks_ms()
    started = last_blink
    while abs(target - _heading(i2c, state)) > TARGET_WINDOW:
        _feed_watchdog()
        if button.value() == 0:
            pixels.off()
            print("Drehung abgebrochen.")
            return False
        if time.ticks_diff(time.ticks_ms(), started) >= TURN_TIMEOUT_MS:
            pixels.off()
            print("Drehziel nicht erreicht.")
            return False
        now = time.ticks_ms()
        if time.ticks_diff(now, last_blink) >= max(80, min(600, int(abs(target - state[0]) * 7))):
            blink_on = not blink_on
            _clear(pixels)
            if blink_on:
                pixels.pixels[indicator] = (255, 180, 0)
            pixels.pixels.write()
            last_blink = now
        time.sleep_ms(5)
    _clear(pixels)
    pixels.pixels[indicator] = (255, 180, 0)
    pixels.pixels.write()
    return True


def _direction_signal(pixels, error):
    _clear(pixels)
    pixels.pixels[10 if error < -20 else 12 if error > 20 else 11] = (255, 0, 0) if error < -20 else (0, 0, 255) if error > 20 else (0, 255, 0)
    pixels.pixels.write()


def _estimated_direction(samples):
    fallback = max(samples, key=lambda item: item[1][3])[0]
    if len(samples) != 3:
        return fallback
    samples.sort(key=lambda item: item[0])
    (x1, first), (x2, second), (x3, third) = samples
    y1, y2, y3 = first[3], second[3], third[3]
    denominator = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
    if denominator == 0:
        return fallback
    target = 0.5 * (x1 * x1 * (y2 - y3) + x2 * x2 * (y3 - y1) + x3 * x3 * (y1 - y2)) / denominator
    curve = -denominator / ((x1 - x2) * (x1 - x3) * (x2 - x3))
    return target if curve < 0 and x1 <= target <= x3 else fallback


def _wifi_direction(i2c, pixels, wlan, button):
    print("WLAN-Richtung: Geraet kurz still halten.")
    gyro_offset, gyro_spread = _calibrate_gyro(i2c)
    print("Gyro-Offset: {:.2f} deg/s | Schwankung: {:.2f} deg/s.".format(gyro_offset, gyro_spread))
    if gyro_spread > GYRO_CALIBRATION_MAX_SPREAD:
        print("Geraet war bei der Kalibrierung nicht still genug. Bitte Richtungsmodus neu starten.")
        pixels.off()
        return
    print("Geradeaus messen.")
    state = [0.0, time.ticks_ms(), gyro_offset]
    samples = []
    bssid = None
    for angle, message, indicator in ((0, "WLAN Eric wird bei 0 Grad gemessen.", None), (90, "90 Grad nach links. LED 12 blinkt schneller beim Ziel.", 12)):
        if indicator is not None and not _turn(i2c, state, angle, pixels, indicator, button):
            return
        print(message if angle == 0 else "WLAN Eric wird bei {:.0f} Grad gemessen.".format(state[0]))
        network = _target_network(wlan, bssid)
        if network:
            bssid = network[1]
            samples.append((state[0] if angle else 0.0, network))
        elif bssid is not None:
            print("Gleicher WLAN-Sender nicht gefunden.")
    print("Zurueck zur Mitte. LED 10 blinkt schneller beim Ziel.")
    if not _turn(i2c, state, 0, pixels, 10, button):
        return
    print("90 Grad nach rechts. LED 10 blinkt schneller beim Ziel.")
    if not _turn(i2c, state, -90, pixels, 10, button):
        return
    print("WLAN Eric wird bei {:.0f} Grad gemessen.".format(state[0]))
    network = _target_network(wlan, bssid)
    if network:
        samples.append((state[0], network))
    elif bssid is not None:
        print("Gleicher WLAN-Sender nicht gefunden.")
    if not samples:
        pixels.off()
        print("WLAN Eric nicht gefunden.")
        return
    target = _estimated_direction(samples)
    best = max(samples, key=lambda item: item[1][3])[1]
    print("RSSI measurements: {}".format(", ".join("{:.0f} degrees: {} dBm".format(angle, network[3]) for angle, network in samples)))
    print("CALCULATED ANGLE: {:.1f} degrees | Best value: {} dBm".format(target, best[3]))
    pressed_at = None
    while True:
        _feed_watchdog()
        _direction_signal(pixels, target - _heading(i2c, state))
        if button.value() == 0:
            pressed_at = time.ticks_ms() if pressed_at is None else pressed_at
            if time.ticks_diff(time.ticks_ms(), pressed_at) >= BUTTON_LONG_PRESS_MS:
                pixels.off()
                return
        else:
            pressed_at = None
        time.sleep_ms(10)


def _comfort(value, ideal_low, ideal_high, bad_low, bad_high):
    if ideal_low <= value <= ideal_high:
        return 1.0
    return max(0.0, (value - bad_low) / (ideal_low - bad_low)) if value < ideal_low else max(0.0, (bad_high - value) / (bad_high - ideal_high))


def _environment(pixels, i2c, button):
    global _dps
    if _dps is None:
        _dps = _Dps3xx(i2c, addr=0x77)
    print("Umwelt: Temperatur, Feuchtigkeit und Schwerkraft bewerten.")
    previous = button.value()
    last_press = time.ticks_ms()
    while True:
        _feed_watchdog()
        temperature, humidity = _read_sht40(i2c)
        temperature += TEMPERATURE_OFFSET_C
        pressure = _dps.measurePressureOnce() / 100
        ax, ay, az = _bmi_values(i2c, 0x0C, 8192)
        gravity = (ax * ax + ay * ay + az * az) ** 0.5
        score = min(_comfort(temperature, 18, 28, -10, 50), _comfort(humidity, 35, 60, 0, 100), _comfort(gravity, 0.8, 1.2, 0.5, 1.8))
        _meter(pixels, ENVIRONMENT_LEDS, score)
        print("Umwelt: {:.1f} C | {:.1f} % | {:.0f} hPa | {:.2f} g | Score {:.0f}%".format(temperature, humidity, pressure, gravity, score * 100))
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < UPDATE_INTERVAL_MS:
            _feed_watchdog()
            pressed, previous, now = _pressed(button, previous, last_press)
            if pressed:
                return now
            time.sleep_ms(20)


def _motion(i2c, pixels, button):
    print("Bewegung: Taste fuer naechsten Modus druecken.")
    previous = button.value()
    last_press = time.ticks_ms()
    while True:
        _feed_watchdog()
        ax, ay, az = _bmi_values(i2c, 0x0C, 8192)
        gx, gy, gz = _bmi_values(i2c, 0x12, 16.384)
        gravity = (ax * ax + ay * ay + az * az) ** 0.5
        rotation = max(abs(gx), abs(gy), abs(gz))
        score = max(0.0, 1.0 - (abs(gravity - 1.0) + rotation / 360.0) / 1.5)
        _meter(pixels, MOTION_LEDS, score)
        print("Bewegung: {:.2f} g | {:.1f} deg/s | Stabilitaet {:.0f}%".format(gravity, rotation, score * 100))
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < 300:
            _feed_watchdog()
            pressed, previous, now = _pressed(button, previous, last_press)
            if pressed:
                return now
            time.sleep_ms(20)


def _doa_emergency_color(confidence, blue_phase):
    score = max(0.0, min(1.0, confidence / 1000.0))
    intensity = int(255 * score)
    return (0, 0, intensity) if blue_phase else (intensity, 0, 0)


def _doa_direction_leds(label):
    if label in (1, 4, 7):
        return (12,)
    if label in (3, 6, 9):
        return (10,)
    if label in (2, 5):
        return (11,)
    return ()


def _audio_direction(pixels, button):
    print("Sirenen-Richtung: LEDs 10-12 zeigen links, vorne/hinten, rechts. Rot-blau blinkend, Helligkeit zeigt die Sicherheit.")
    previous = button.value()
    last_press = time.ticks_ms()
    last_label = -2
    while True:
        _feed_watchdog()
        label, confidence = _doa_state
        _clear(pixels)
        if 0 < label < len(DOA_LABELS) and label != 8 and confidence:
            blue_phase = (time.ticks_ms() // 250) % 2 == 0
            for index in _doa_direction_leds(label):
                pixels.pixels[index] = _doa_emergency_color(confidence, blue_phase)
            if label != last_label:
                print("Audio-DoA: {} | Confidence {:.1f}%".format(DOA_LABELS[label], confidence / 10.0))
        pixels.pixels.write()
        last_label = label
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < 100:
            _feed_watchdog()
            pressed, previous, now = _pressed(button, previous, last_press)
            if pressed:
                return now
            time.sleep_ms(20)


def main(bmi_configured=False, i2c=None):
    global _watchdog
    gc.collect()
    print("BMI270 wird initialisiert.")
    led = machine.Pin(LED_PIN, machine.Pin.OUT)
    led.off()
    button = machine.Pin(BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    if i2c is None:
        i2c = machine.I2C(0, scl=machine.Pin("P8_0"), sda=machine.Pin("P8_1"), freq=400000)
    devices = i2c.scan()
    if SHT40_ADDRESS not in devices or 0x77 not in devices:
        raise OSError("SHT40 oder DPS368 nicht gefunden")
    if not bmi_configured:
        _configure_bmi(i2c)
    pixels = _Pixels(machine.Pin(NEOPIXEL_PIN))
    wlan = network.WLAN(network.STA_IF)
    _watchdog = machine.WDT(timeout=WIFI_WATCHDOG_MS)
    _start_doa()
    actions = (lambda: _wifi_strength(i2c, pixels, wlan, button), lambda: _environment(pixels, i2c, button), lambda: _motion(i2c, pixels, button), lambda: _audio_direction(pixels, button))
    labels = ("WLAN-Praesenz", "Umwelt", "Bewegung", "Sirenen-Richtung")
    mode = 0
    print("Bereit. Taste P17_1 wechselt den Modus.")
    while True:
        print("Modus: {}".format(labels[mode]))
        try:
            actions[mode]()
        except Exception as error:
            print("Fehler: {}".format(error))
            time.sleep_ms(500)
        mode = (mode + 1) % len(actions)
