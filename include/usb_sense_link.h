#include <sensors.h>
#include <lms/module.h>
#include <termios.h>

class UsbSenseLink:public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
private:
    const lms::type::ModuleConfig* config;
    int usb_fd;
    struct termios usb_tio;
};
