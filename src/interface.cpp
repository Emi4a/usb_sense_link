#include <usb_sense_link.h>

extern "C" {
void* getInstance () {
    return new UsbSenseLink();
}
}
