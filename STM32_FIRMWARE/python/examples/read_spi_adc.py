#!/usr/bin/env python3
"""
Example: poll the STM32F103 ITV valve ADC feedback over SPI (CAN 0x500-0x503).

Usage::

    python read_spi_adc.py                 # built-in mock (no hardware)
    python read_spi_adc.py COM3            # real STM32 over USB CDC
    python read_spi_adc.py COM3 20         # 20 Hz (default 10 Hz)
"""

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from tue_usb import TueUsbClient


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "MOCK_PORT"
    use_mock = port == "MOCK_PORT"
    rate_hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    period = 1.0 / max(rate_hz, 0.1)

    print(f"Connecting to {port} (mock={use_mock})...")
    with TueUsbClient(port=port, mock=use_mock) as client:
        print(f"[CTRL] {client.get_spi_adc_status()}")
        print("Polling all five channels; Ctrl+C to stop.")
        try:
            while True:
                fb = client.read_spi_adc()
                mv = fb.millivolts()
                values = "  ".join(
                    f"CH{i}={raw:4d}/{volts:7.1f}mV"
                    for i, (raw, volts) in enumerate(zip(fb.channels, mv))
                )
                print(
                    f"seq={fb.seq:3d} status=0x{fb.status:02X} "
                    f"ok={fb.rx_ok_cnt:5d} crc={fb.crc_err_cnt:3d} | {values}",
                    flush=True,
                )
                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopped")


if __name__ == "__main__":
    main()
