"""Flash the complete Survival Sensor Box stack to a KIT_PSE84_AI board.

The package contains matching prebuilt CM55 and CM33 images plus the compiled
MicroPython application, so users do not need Deepcraft or mpy-cross installed.
"""

import argparse
import base64
import subprocess
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as error:
    raise SystemExit("Missing dependency. Run: py -m pip install -r requirements.txt") from error


ROOT = Path(__file__).resolve().parent
FIRMWARE_DIR = ROOT / "firmware"
APP_DIR = ROOT / "app"
OPENOCD_DIR = ROOT / "tools" / "openocd"
OPENOCD = OPENOCD_DIR / "bin" / "openocd.exe"
OPENOCD_SCRIPTS = OPENOCD_DIR / "scripts"
CM55_IMAGE = FIRMWARE_DIR / "cm55_deepcraft.hex"
CM33_IMAGE = FIRMWARE_DIR / "cm33_micropython.hex"
QSPI_FLASHLOADER = FIRMWARE_DIR / "PSE84_SMIF.FLM"
QSPI_CONFIG = FIRMWARE_DIR / "qspi_config.cfg"
CHUNK_SIZE = 300
SERIAL_BAUDRATE = 115200
PROMPT_TIMEOUT_SECONDS = 8
PORT_REENUMERATION_TIMEOUT_SECONDS = 20
KITPROG3_VID = 0x04B4
KITPROG3_PID = 0xF155


def require_file(path, description):
    if not path.is_file():
        raise RuntimeError("{} not found: {}".format(description, path))


def tcl_path(path):
    """Quote a Windows path for the Tcl command interpreter used by OpenOCD."""
    return "{{{}}}".format(str(path.resolve()).replace("\\", "/"))


def verify_openocd_assets():
    require_file(OPENOCD, "OpenOCD executable")
    require_file(OPENOCD_SCRIPTS / "interface" / "kitprog3.cfg", "KitProg3 configuration")
    require_file(OPENOCD_SCRIPTS / "target" / "infineon" / "pse84xgxs2.cfg", "PSoC Edge configuration")
    require_file(QSPI_FLASHLOADER, "PSE84 SMIF flash loader")
    require_file(QSPI_CONFIG, "PSE84 SMIF bank layout")


def kitprog3_port(port_name):
    for port in list_ports.comports():
        if port.device.upper() == port_name.upper():
            if port.vid != KITPROG3_VID or port.pid != KITPROG3_PID:
                raise RuntimeError("{} is not a KitProg3 USB-UART port.".format(port_name))
            if not port.serial_number:
                raise RuntimeError("Could not read the KitProg3 serial number for {}.".format(port_name))
            return port
    raise RuntimeError("{} is not currently available.".format(port_name))


def wait_for_kitprog3_port(probe_serial, previous_port):
    deadline = time.monotonic() + PORT_REENUMERATION_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        matches = [
            port for port in list_ports.comports()
            if port.vid == KITPROG3_VID and port.pid == KITPROG3_PID and port.serial_number == probe_serial
        ]
        if len(matches) == 1:
            port_name = matches[0].device
            if port_name.upper() != previous_port.upper():
                print("KitProg3 USB-UART moved from {} to {} after reset.".format(previous_port, port_name))
            return port_name
        if len(matches) > 1:
            raise RuntimeError("More than one KitProg3 USB-UART port matches serial {}.".format(probe_serial))
        time.sleep(0.25)
    raise RuntimeError(
        "KitProg3 USB-UART with serial {} did not reappear after CM33 flashing. "
        "Reconnect the board, find its COM port, then rerun with --skip-cm55 --skip-cm33."
        .format(probe_serial)
    )


def flash_image(image, description, probe_serial=None):
    verify_openocd_assets()
    require_file(image, description)
    command = [
        str(OPENOCD),
        "-s", str(OPENOCD_SCRIPTS),
        "-s", str(FIRMWARE_DIR),
        "-c", "set ENABLE_CM55 1",
        "-c", "set QSPI_FLASHLOADER {}".format(tcl_path(QSPI_FLASHLOADER)),
        "-c", "source [find interface/kitprog3.cfg]",
    ]
    if probe_serial:
        command.extend(("-c", "adapter serial {}".format(probe_serial)))
    command.extend((
        "-c", "transport select swd",
        "-c", "source [find target/infineon/pse84xgxs2.cfg]",
        "-c", "init",
        "-c", "reset init",
        "-c", "adapter speed 12000",
        "-c", "targets cat1d.cm33",
        "-c", "flash write_image erase {}".format(tcl_path(image)),
        "-c", "verify_image {}".format(tcl_path(image)),
        "-c", "reset run; shutdown",
    ))
    subprocess.run(command, check=True)


def read_until(port, marker, timeout=PROMPT_TIMEOUT_SECONDS, occurrences=1):
    received = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        if data:
            received.extend(data)
            if received.count(marker) >= occurrences:
                return bytes(received)
    raise RuntimeError("No response from the MicroPython Raw REPL: {!r}".format(bytes(received)))


def enter_raw_repl(port):
    port.write(b"\x02")
    port.flush()
    try:
        read_until(port, b">>>", timeout=1)
    except RuntimeError:
        port.write(b"\r\x03\x03")
        port.flush()
        read_until(port, b">>>", timeout=PROMPT_TIMEOUT_SECONDS)
    port.write(b"\x01")
    port.flush()
    response = read_until(port, b">", timeout=PROMPT_TIMEOUT_SECONDS)
    if b"raw REPL" not in response and b"\r\n>" not in response:
        raise RuntimeError("Could not activate the Raw REPL: {!r}".format(response))


def execute(port, code):
    port.write(code.encode("ascii") + b"\x04")
    port.flush()
    response = read_until(port, b"\x04", timeout=PROMPT_TIMEOUT_SECONDS, occurrences=2)
    ok_offset = response.find(b"OK")
    if ok_offset < 0:
        raise RuntimeError("MicroPython rejected the command: {!r}".format(response))
    stdout, _, stderr = response[ok_offset + 2:].partition(b"\x04")
    stderr = stderr.strip(b"\x04> \r\n")
    if stderr:
        raise RuntimeError(stderr.decode("utf-8", "replace"))
    return stdout.decode("utf-8", "replace").strip()


def upload_file(port, local_path, remote_path):
    encoded = base64.b64encode(local_path.read_bytes()).decode("ascii")
    execute(port, "import gc,ubinascii\ngc.collect()\nf=open({!r}, 'wb')".format(remote_path))
    for offset in range(0, len(encoded), CHUNK_SIZE):
        execute(port, "f.write(ubinascii.a2b_base64({!r}))".format(encoded[offset:offset + CHUNK_SIZE]))
    execute(port, "f.close()\ndel f\ngc.collect()")
    actual_size = int(execute(port, "import os\nprint(os.stat({!r})[6])".format(remote_path)))
    expected_size = local_path.stat().st_size
    if actual_size != expected_size:
        raise RuntimeError("{} has {} bytes instead of {}".format(remote_path, actual_size, expected_size))
    print("OK  {:<24} {} bytes".format(remote_path, actual_size))


def upload_application(port_name):
    files = (
        (APP_DIR / "main.py", "/main.py"),
        (APP_DIR / "bmi270_config.bin", "/bmi270_config.bin"),
        (APP_DIR / "survival_config.py", "/survival_config.py"),
        (APP_DIR / "device_main.mpy", "/device_main.mpy"),
    )
    for local_path, remote_path in files:
        require_file(local_path, "Application file")
    print("Connecting to {}...".format(port_name))
    try:
        with serial.Serial(port_name, SERIAL_BAUDRATE, timeout=0.1) as port:
            enter_raw_repl(port)
            print(execute(port, "import sys\nprint(getattr(sys.implementation, '_mpy', sys.implementation))"))
            for local_path, remote_path in files:
                upload_file(port, local_path, remote_path)
            port.write(b"\x02")
            port.flush()
    except serial.SerialException as error:
        raise RuntimeError(
            "Could not open {}. Close Thonny, VS Code Serial Monitor, or any other serial terminal, "
            "then rerun with --skip-cm55 --skip-cm33. ({})".format(port_name, error)
        ) from error


def main():
    parser = argparse.ArgumentParser(
        description="Flash the complete Survival Sensor Box stack to a KIT_PSE84_AI board."
    )
    parser.add_argument("--port", help="KitProg3 USB-UART port, for example COM68")
    parser.add_argument("--probe-serial", help="KitProg3 serial number; normally detected from --port")
    parser.add_argument("--skip-cm55", action="store_true", help="Do not flash the CM55 Deepcraft image")
    parser.add_argument("--skip-cm33", action="store_true", help="Do not flash the CM33 MicroPython image")
    parser.add_argument("--skip-app", action="store_true", help="Do not upload the MicroPython application")
    arguments = parser.parse_args()

    if arguments.skip_cm55 and arguments.skip_cm33 and arguments.skip_app:
        parser.error("At least one deployment step must be enabled.")
    if not arguments.skip_app and not arguments.port:
        parser.error("--port is required when uploading the MicroPython application.")

    probe_serial = arguments.probe_serial
    if arguments.port:
        port = kitprog3_port(arguments.port)
        if probe_serial and probe_serial != port.serial_number:
            parser.error("--probe-serial does not match the KitProg3 connected to {}.".format(arguments.port))
        probe_serial = port.serial_number

    try:
        if not arguments.skip_cm55:
            print("[1/3] Flashing CM55 Deepcraft image...")
            flash_image(CM55_IMAGE, "CM55 Deepcraft image", probe_serial)
        if not arguments.skip_cm33:
            print("[2/3] Flashing CM33 MicroPython image...")
            flash_image(CM33_IMAGE, "CM33 MicroPython image", probe_serial)
        if not arguments.skip_app:
            if not arguments.skip_cm33:
                arguments.port = wait_for_kitprog3_port(probe_serial, arguments.port)
            print("[3/3] Uploading MicroPython application...")
            upload_application(arguments.port)
    except (OSError, RuntimeError, subprocess.CalledProcessError, serial.SerialException) as error:
        print("Deployment failed: {}".format(error), file=sys.stderr)
        raise SystemExit(1)

    print("Deployment complete. Reset the board and press the button within 4 seconds to start.")


if __name__ == "__main__":
    main()