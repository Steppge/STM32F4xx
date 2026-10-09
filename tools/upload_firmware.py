#!/usr/bin/env python3
"""
upload_firmware.py - copy firmware.bin to the SD card in the BTT Octopus Pro over USB

grblHAL receives the file with YModem (SDCARD_ENABLE 2) and stores it on the SD card.
After a reboot the BTT bootloader flashes firmware.bin and renames it to FIRMWARE.CUR.

  python tools/upload_firmware.py               upload only
  python tools/upload_firmware.py --reboot      upload, then reboot to flash it
  python tools/upload_firmware.py --reboot-only reboot to flash a previously uploaded file

The machine must be idle and no other program (LightBurn) may have the port open.
Requires pyserial (pip install pyserial).
"""

import argparse
import os
import sys
import time

import serial
import serial.tools.list_ports

USB_VID, USB_PID = 0x0483, 0x5740   # STM32 virtual COM port
FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '.pio', 'build', 'octopus_pro_f446', 'firmware.bin')
TARGET = 'firmware.bin'

SOH, STX, EOT, ACK, NAK, CAN, CRC = 0x01, 0x02, 0x04, 0x06, 0x15, 0x18, 0x43


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            return p.device
    return None


def open_port(timeout_s=0):
    """Open the board's COM port, optionally waiting for it to appear (after a reboot)."""
    end = time.time() + timeout_s
    while True:
        port = find_port()
        if port:
            try:
                return serial.Serial(port, 115200, timeout=1)
            except serial.SerialException as e:
                if time.time() >= end:
                    sys.exit(f'{port} could not be opened ({e}). Is LightBurn still connected?')
        elif time.time() >= end:
            sys.exit('Board not found on USB.')
        time.sleep(0.5)


def command(ser, cmd, timeout=3.0):
    """Send a line, return the response lines up to ok/error."""
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    lines, end = [], time.time() + timeout
    while time.time() < end:
        line = ser.readline().decode(errors='replace').strip()
        if line:
            lines.append(line)
            if line == 'ok' or line.startswith('error'):
                break
    return lines


def status(ser):
    ser.reset_input_buffer()
    ser.write(b'?')
    end = time.time() + 2
    while time.time() < end:
        line = ser.readline().decode(errors='replace').strip()
        if line.startswith('<'):
            return line
    return ''


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def wait_reply(ser, timeout=5.0):
    """Wait for ACK, NAK or CAN, other characters are ignored."""
    end = time.time() + timeout
    while time.time() < end:
        c = ser.read(1)
        if c and c[0] in (ACK, NAK, CAN):
            return c[0]
    return None


def send_packet(ser, num, payload, size):
    data = payload.ljust(size, b'\x1a' if num else b'\x00')
    packet = bytes([STX if size == 1024 else SOH, num & 0xFF, 0xFF - (num & 0xFF)]) + data + crc16(data).to_bytes(2, 'big')
    for _ in range(10):
        ser.write(packet)
        reply = wait_reply(ser)
        if reply == ACK:
            return True
        if reply == CAN:
            sys.exit('Upload cancelled by the board (file could not be opened?).')
    sys.exit(f'No acknowledge for packet {num}, upload aborted.')


def upload(ser, path):
    data = open(path, 'rb').read()
    print(f'Uploading {os.path.abspath(path)} ({len(data)} bytes) as {TARGET} ...')

    send_packet(ser, 0, f'{TARGET}\0{len(data)}\0'.encode(), 128)
    for i in range(0, len(data), 1024):
        send_packet(ser, i // 1024 + 1, data[i:i + 1024], 1024)
        print(f'\r  {min(i + 1024, len(data)) * 100 // len(data):3d} %', end='', flush=True)
    print()

    for _ in range(3):              # end of file
        ser.write(bytes([EOT]))
        if wait_reply(ser) == ACK:
            break
    send_packet(ser, 0, b'', 128)   # end of batch
    time.sleep(0.5)

    # check the file on the card
    for line in command(ser, '$F+'):
        if TARGET in line.lower():
            print('  on card: ' + line)
            if f'SIZE:{len(data)}' not in line:
                sys.exit('File size on the card does not match, do not reboot!')
            return len(data)
    sys.exit(f'{TARGET} not found on the card after upload, do not reboot!')


def reboot(ser):
    print('Rebooting, the bootloader flashes the firmware ...')
    try:
        ser.write(b'$REBOOT\n')
        time.sleep(0.5)
    finally:
        ser.close()
    time.sleep(5)
    ser = open_port(timeout_s=40)
    time.sleep(1)
    info = [l for l in command(ser, '$I') if l.startswith(('[VER', '[BOARD', '[PLUGIN'))]
    print('Board is back:')
    for line in info:
        print('  ' + line)
    ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reboot', action='store_true', help='reboot after the upload to flash the firmware')
    ap.add_argument('--reboot-only', action='store_true', help='only reboot (file already on the card)')
    ap.add_argument('--file', default=FIRMWARE, help='firmware file (default: PlatformIO build output)')
    args = ap.parse_args()

    ser = open_port()
    time.sleep(0.3)
    st = status(ser)
    if not st.startswith('<Idle'):
        ser.close()
        sys.exit(f'Machine is not idle ({st or "no status"}), nothing done.')

    if not args.reboot_only:
        if not os.path.isfile(args.file):
            sys.exit(f'{args.file} not found.')
        command(ser, '$FM', 5)  # mount the SD card (not mounted automatically)
        upload(ser, args.file)

    if args.reboot or args.reboot_only:
        reboot(ser)
    else:
        ser.close()
        print('Upload complete. Reboot the board (or run with --reboot-only) to flash it.')


if __name__ == '__main__':
    main()
