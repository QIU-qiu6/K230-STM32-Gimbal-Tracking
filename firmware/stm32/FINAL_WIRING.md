# X42S TTL wiring correction

The official X42S manual (page 26) defines the TTL wiring as follows:

- Yaw: STM32 PA9 TX -> motor R/A/H; motor T/B/L -> STM32 PA10 RX.
- Pitch: STM32 PB10 TX -> motor R/A/H; motor T/B/L -> STM32 PB11 RX.
- STM32 and both motors must share GND.

The inherited handoff document had R/A/H and T/B/L reversed. Do not restore that old wiring.

This project is the normal runtime build. It enables both motors after successful self-test and starts tracking when valid K230 target coordinates are received.
