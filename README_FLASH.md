# Flashing with Black Magic Probe

Use the shell wrapper; no GDB Python required.

```bash
west build -b nrf52dk_nrf52810 .
./flash-bmp.sh /dev/cu.usbmodemXXXX1
```

Tip: The Black Magic Probe exposes two serial devices. Use the one that ends in `...1` for GDB (the other is UART/CDC).
