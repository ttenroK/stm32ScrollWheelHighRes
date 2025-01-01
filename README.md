# stm32ScrollWheelHighRes

This project implements an HID device with high resolution scrolling for the STM32 Black Pill.

A AS5600 magnetic encoder module is used for detecting the position of the knob.

The HID device already provides further functionality, prepared in the device descriptor. Beside the scrolling (mouse) functions, controlling the volume and media (skip track, play pause) are already implemented.
All functions (including high-resolution scrolling) work across all platforms. Both Windows and Linux have already been tested.
