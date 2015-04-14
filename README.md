# usb_sense_link

[LMS Module](https://github.com/Phibedy/LMS)

## Communication via USB

Microcontroller <-> Computer

Uses the [sense_link](https://github.com/Bitfroest/sense_link/) communication protocol

### Handle Errors
- full byte buffer
- input/output
- not enought bytes read/wrote
- connection problems - usb re-initalisation
- bootloader startup time

### Config
- path = usb device address
