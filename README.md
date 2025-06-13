# hf-hal
Contains the hal layer - as used in the HardFOC controller

## WS2812 Driver Integration

The HAL now pulls in the optional `hf-ws2812-rmt-driver` component automatically
via `EXTRA_COMPONENT_DIRS`. When this repository is used as a component the
driver's headers (such as `ws2812_cpp.hpp`) are available without any extra
configuration.

If you need to override the path, set `EXTRA_COMPONENT_DIRS` before running
`idf.py`:

```
export EXTRA_COMPONENT_DIRS=$PWD/utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver
```
