#include <usb_sense_link.h>
#include <string>

#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

bool UsbSenseLink::initialize(){
    config = getConfig();
    std::string path = config->get<std::string>("path");

    usb_fd = open(path.c_str(), O_RDWR | O_NOCTTY );
    if (usb_fd < 0) {
        logger.perror("init") << "Open Senseboard";
        return false;
    }

    ///Termios
    //further reading http://en.wikibooks.org/wiki/Serial_Programming/termios
    tcgetattr(usb_fd, &usb_tio);

    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    usb_tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // usb_tio.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    usb_tio.c_oflag = 0;
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    usb_tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    usb_tio.c_cflag &= ~(CSIZE | PARENB);
    usb_tio.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    usb_tio.c_cc[VMIN]  = 1;
    usb_tio.c_cc[VTIME] = 0;

    // first set speed and then set attr!!!
    if(cfsetispeed(&usb_tio, B115200) < 0 || cfsetospeed(&usb_tio, B115200) < 0) {
        logger.perror("init") << "Baud rate";

    }

    if(tcsetattr(usb_fd, TCSANOW, &usb_tio) < 0) {
        logger.perror("init") << "SET ATTR";
    }

    return true;
}

bool UsbSenseLink::deinitialize(){
    logger.info("deinitialize") << "close Arduino";

    close(usb_fd);

    return true;
}


bool UsbSenseLink::cycle(){
    sleep(1);
    Message m;
    static char c = 'A';
    static bool ledValue = false;
    m.mType = SENSOR_DATA;
    m.sType = LED;
    m.id = 1;
    m.sensorData.Led.value = ledValue ? ON : OFF;
    ledValue = !ledValue;

    c++;
    char buffer[4];
    logger.info("cycle") << encode(&m, buffer);
    if(write(usb_fd, &c, 1) != 1){
        logger.perror("cycle");
    } else {
        logger.debug("cycle") << "Send finished:" << c;
    }
    if(read(usb_fd,buffer,1) != 1){
        logger.perror("cycle");
    }else {
        logger.debug("cycle") << "Read finished:" << buffer[0];
    }
    //sleep(10);
    return true;
}
