"""Motor COM-port test / monitor for the STM32 USB CDC link.

Default mode is a raw monitor of whatever the STM32 sends over USB CDC.

Close the unified GUI first: it owns the motor COM port.

Examples
--------
    python test.py                        # follow COM14 @ 115200
    python test.py --port COM7            # different port
    python test.py --seconds 30           # stop after 30 s
    python test.py --hex                  # show raw bytes in hex
    python test.py --send "hello"         # send ASCII (generic serial test)
    python test.py --send-hex "AA 55 01 00 03 00 00 08"
    python test.py --list                 # list serial ports
    python test.py --ping                 # USB SYS_PING (0x100)
    python test.py --haptic-ping          # STM32 -> ESP32 binary ping (0x301)
    python test.py --info                 # USB board/info/stepper status
    python test.py --diag                 # USB haptic UART diagnostics (0x302)

Note: `--ping` is the USB board ping (CAN 0x100). `--haptic-ping` performs the
STM32 master 20-byte binary ping over USART2 at 115200 baud (CAN 0x301) and
prints the ESP32 status reply.  `--info` and `--diag` are USB-only checks.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover - environment dependent
    raise SystemExit(f"pyserial is required: {exc}")

DEFAULT_PORT = "COM14"
DEFAULT_BAUD = 115200

# The project keeps the TUE client in a plain directory beside this file.
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
REFERENCE = os.path.join(REPO_ROOT, "python_reference")
if REFERENCE not in sys.path:
    sys.path.insert(0, REFERENCE)


def list_serial_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports found")
        return 1
    for port in ports:
        print(f"{port.device:12s} {port.description}")
    return 0


def open_serial(port: str, baud: int, timeout: float) -> "serial.Serial":
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    try:
        ser.dtr = True
        ser.rts = True
    except Exception:
        pass
    return ser


def _parse_hex(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    return bytes(int(piece, 16) for piece in cleaned.split())


def monitor(
    port: str,
    baud: int,
    timeout: float,
    seconds: Optional[float],
    hex_mode: bool,
    send: Optional[str],
    send_hex: Optional[str],
) -> int:
    try:
        ser = open_serial(port, baud, timeout)
    except Exception as exc:
        print(f"cannot open {port}: {exc}")
        return 1

    print(f"opened {port} @ {baud} 8N1 (Ctrl+C to stop)")
    try:
        if send_hex:
            payload = _parse_hex(send_hex)
            ser.write(payload)
            ser.flush()
            print(f"TX {len(payload)} B: {payload.hex(' ')}")
        if send is not None:
            payload = (send + "\n").encode()
            ser.write(payload)
            ser.flush()
            print(f"TX {len(payload)} B: {send}")
    except Exception as exc:
        print(f"write failed: {exc}")
        ser.close()
        return 1

    deadline = None if seconds is None else time.monotonic() + seconds
    pending = bytearray()

    try:
        while deadline is None or time.monotonic() < deadline:
            waiting = ser.in_waiting
            data = ser.read(waiting if waiting else 1)
            if not data:
                continue

            if hex_mode:
                print(data.hex(" "), flush=True)
                continue

            pending.extend(data)
            while b"\n" in pending:
                line, _, rest = pending.partition(b"\n")
                pending = bytearray(rest)
                text = line.rstrip(b"\r").decode("utf-8", "replace")
                print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)
    except KeyboardInterrupt:
        print("\nstopped")
    except serial.SerialException as exc:
        print(f"serial error: {exc}")
        return 1
    finally:
        if pending and not hex_mode:
            text = pending.rstrip(b"\r").decode("utf-8", "replace")
            if text:
                print(f"[{time.strftime('%H:%M:%S')}] {text}")
        try:
            ser.close()
        except Exception:
            pass
    return 0


def protocol_check(port: str, baud: int, timeout: float, action: str) -> int:
    try:
        from tue_usb import TueUsbClient, TueUsbError  # noqa: PLC0415
    except Exception as exc:
        print(f"cannot import tue_usb: {exc}")
        return 1

    try:
        with TueUsbClient(port, baudrate=baud, timeout=max(timeout, 0.35)) as client:
            if action == "ping":
                token = client.ping(0x12345678)
                print(f"SYS_PING (0x100): echo=0x{token:08X}")
            elif action == "info":
                info = client.get_info()
                print(
                    f"SYS_INFO: hw={info.hw_id}.{info.hw_rev} "
                    f"fw={info.version_string} flags=0x{info.build_flags:08X}"
                )
                print(f"SYS_UPTIME: {client.get_uptime_ms()} ms")
                status = client.step_get_status()
                print(
                    f"STEP_STATUS: enabled={status.is_enabled} "
                    f"busy={status.is_busy} queue={status.queue_depth}"
                )
            elif action == "haptic_ping":
                try:
                    pong = client.haptic_ping(0x5A)
                    print(f"HAPTIC PING (0x301): {pong}")
                    try:
                        diag = client.haptic_uart_diag()
                        print(f"HAPTIC UART DIAG (0x302): {diag}")
                    except Exception:
                        pass
                except TueUsbError as exc:
                    print(f"MCU error: {exc}")
                    try:
                        diag = client.haptic_uart_diag()
                        print(f"HAPTIC UART DIAG (0x302): {diag}")
                    except Exception:
                        pass
                    return 1
            elif action == "diag":
                diag = client.haptic_uart_diag()
                print(f"HAPTIC UART DIAG (0x302): {diag}")
            else:  # pragma: no cover - argparse prevents this
                print(f"unknown action {action!r}")
                return 1
    except TueUsbError as exc:
        print(f"MCU error: {exc}")
        return 1
    except Exception as exc:
        print(f"{action} failed: {exc}")
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="STM32 motor COM-port test / monitor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"serial port (default {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"baud rate (default {DEFAULT_BAUD}; USB CDC ignores it)")
    parser.add_argument("--timeout", type=float, default=0.2,
                        help="serial read timeout in seconds (default 0.2)")
    parser.add_argument("--seconds", type=float, default=None,
                        help="stop the monitor after N seconds")
    parser.add_argument("--hex", action="store_true",
                        help="print received bytes in hex instead of text")
    parser.add_argument("--send", metavar="TEXT",
                        help="send an ASCII line (generic serial test; the STM32 "
                             "binary parser ignores plain ASCII)")
    parser.add_argument("--send-hex", metavar="BYTES",
                        help="send raw bytes, e.g. 'AA 55 01 00 03 00 00 08'")
    parser.add_argument("--list", action="store_true",
                        help="list serial ports and exit")

    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--ping", action="store_true",
                         help="USB SYS_PING (CAN 0x100)")
    actions.add_argument("--haptic-ping", action="store_true",
                         help="STM32 master 20B binary ping over USART2 @ 115200 (CAN 0x301)")
    actions.add_argument("--info", action="store_true",
                         help="USB SYS_INFO / uptime / stepper status")
    actions.add_argument("--diag", action="store_true",
                         help="USB haptic UART diagnostics (CAN 0x302)")
    return parser


def main(argv: Optional[list] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list:
        return list_serial_ports()

    if args.ping or args.haptic_ping or args.info or args.diag:
        if args.ping:
            action = "ping"
        elif args.haptic_ping:
            action = "haptic_ping"
        elif args.info:
            action = "info"
        else:
            action = "diag"
        return protocol_check(args.port, args.baud, args.timeout, action)

    return monitor(
        args.port,
        args.baud,
        args.timeout,
        args.seconds,
        args.hex,
        args.send,
        args.send_hex,
    )


if __name__ == "__main__":
    sys.exit(main())
