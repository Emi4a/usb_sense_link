#include <usb_sense_link.h>
#include <string>

#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

const int UsbSenseLink::MAX_LOOP_COUNT = 100;

bool UsbSenseLink::initialize(){
    config = getConfig();
    std::string path = config->get<std::string>("path");

    usb_fd = open(path.c_str(), O_RDWR | O_NOCTTY);
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

    /*
     * http://arduino.cc/en/Main/ArduinoBoardNano
     *
     * When the Nano is connected to either a computer running
     * Mac OS X or Linux, it resets each time a connection is made
     * to it from software (via USB). For the following half-second
     * or so, the bootloader is running on the Nano.
     *
     * [...] make sure that the software with which it communicates
     * waits a second after opening the connection and before sending
     * this data.
     */
    sleep(3);

    return true;
}

bool UsbSenseLink::deinitialize(){
    logger.info("deinitialize") << "close Arduino";

    close(usb_fd);

    return true;
}

bool UsbSenseLink::readFull(char* buffer, int bufSize) {
    int result, loopCount = MAX_LOOP_COUNT;

    while(loopCount -- > 0) {
        result = read(usb_fd, buffer, bufSize);
        if(result != -1) {
            // if there was no error while reading
            // then go further in the buffer
            buffer += result;
            bufSize -= result;
        } else {
            // we will skip errors here
        }

        // check if the full bufer was read
        if(0 == bufSize) return true;
    }

    // in case MAX_LOOP_COUNT was reached
    return false;
}

bool UsbSenseLink::writeFull(const char* buffer, int bufSize) {
    int result, loopCount = MAX_LOOP_COUNT;

    while(loopCount -- > 0) {
        result = write(usb_fd, buffer, bufSize);
        if(result != -1) {
            buffer += result;
            bufSize -= result;
        } else {
            // skip errors
        }

        if(0 == bufSize) return true;
    }

    return false;
}

bool UsbSenseLink::cycle(){
    //usleep(10000);
    sense_link::Message m;
    static char c = 'A';
    static bool ledValue = false;
    m.mType = sense_link::SENSOR_DATA;
    m.sType = sense_link::LED;
    m.id = 1;
    m.sensorData.Led.value = ledValue ? sense_link::ON : sense_link::OFF;
    ledValue = !ledValue;

    //c++;
    char buffer[4];
    logger.info("cycle") << encode(&m, buffer);
    if(writeFull(&c, 1) != 1){
        logger.perror("cycle");
    } else {
        logger.debug("cycle") << "Send finished:" << c;
    }
    if(readFull(buffer,1) != 1){
        logger.perror("cycle");
    }else {
        logger.debug("cycle") << "Read finished:" << buffer[0];
    }
    //sleep(10);
    return true;
}
