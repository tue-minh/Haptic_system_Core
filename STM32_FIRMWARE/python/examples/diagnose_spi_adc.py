#!/usr/bin/env python3
"""
Diagnose the STM32 master <-> STM32F103 slave SPI link (CAN 0x504).

`read_spi_adc.py` reports the *decoded* values, which are all zero when no
frame ever parses. This tool shows the bytes the master actually clocked in,
which is what distinguishes the usual causes:

    AA 55 ...          -> frames arriving; the link works
    00 00 00 ...       -> MISO stuck low   (slave not driving, no power, no GND)
    FF FF FF ...       -> MISO floating    (slave not driving / not configured)
    xx xx ... random   -> clocking or mode mismatch (CPOL/CPHA, wrong pins)

Usage::

    python diagnose_spi_adc.py              # built-in mock (no hardware)
    python diagnose_spi_adc.py COM3         # real STM32 over USB CDC
    python diagnose_spi_adc.py COM3 5       # sample every 5 s
"""

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import TueUsbClient  # noqa: E402
from tue_usb.defs import CanId  # noqa: E402

CAN_ADC_RAW = int(CanId.SENSOR_ADC_RAW)
CAN_ADC_META = int(CanId.SENSOR_ADC_CH4_META)


def classify(raw: bytes) -> str:
    """Turn the received bytes into a likely cause."""
    if len(raw) >= 2 and raw[0] == 0xAA and raw[1] == 0x55:
        return "OK - SOF present, frames are arriving"
    if raw and all(b == 0x00 for b in raw):
        return "MISO stuck LOW - slave not driving (power, GND, or CS not reaching PB6)"
    if raw and all(b == 0xFF for b in raw):
        return "MISO floating HIGH - slave not driving (check slave power/GND, SPI1 pins)"
    return "unexpected pattern - check CPOL/CPHA, wiring, and that CS1=PB8 -> PB6"


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "MOCK_PORT"
    use_mock = port == "MOCK_PORT"
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    print(f"Connecting to {port} (mock={use_mock})...")
    with TueUsbClient(port=port, mock=use_mock) as client:
        print(f"[CTRL] {client.get_spi_adc_status()}")
        print(f"Reading 0x{CAN_ADC_RAW:03X} (raw bytes) + 0x{CAN_ADC_META:03X} "
              f"(counters); Ctrl+C to stop.\n")
        try:
            while True:
                resp = client.transact(CAN_ADC_RAW, b"")
                raw = bytes(resp.payload[:8])

                meta = client.transact(CAN_ADC_META, b"")
                ok = int.from_bytes(meta.payload[4:6], "little")
                bad = int.from_bytes(meta.payload[6:8], "little")
                seq = meta.payload[2]
                status = meta.payload[3]

                print(f"raw  = {raw.hex(' ').upper()}")
                print(f"       {classify(raw)}")
                print(f"seq={seq:3d} status=0x{status:02X} rx_ok={ok:5d} bad={bad:5d}")
                print("-" * 60, flush=True)
                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopped")


if __name__ == "__main__":
    main()
