#include <usb_sense_link.h>
#include <string>

#include <unistd.h>

#include <cmath>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

#include "lms/datamanager.h"

const int UsbSenseLink::MAX_LOOP_COUNT = 1000;

bool UsbSenseLink::initialize(){
    config = getConfig();
    path = config->get<std::string>("path");
    initUSB();

    // open sense board data channel
    senseBoard = datamanager()
                ->readChannel<sense_link::SenseBoard>(this, "SENSE_BOARD");
    return true;
}

bool UsbSenseLink::is_valid_fd(int fd) {
    /// Sende Anfrage damit ioctl errno neu setzt
    int rc = ioctl(fd, USBDEVFS_CONNECTINFO, 0);

    /// Wenn rc != 0 gab es einen Fehler
    if (rc == 0){
        return true;
    }

    logger.perror("is_valid_fd") << "Error in ioctl";
    /// Haben wir einen Input/Output error? -> usb neu initialisieren
    if(errno == EIO) {
        close(usb_fd);
        while(!initUSB()) {
            usleep(100);
        }
    }

    return false;
}

bool UsbSenseLink::tooMuchBytesAvailable() {
    uint bytes_available;
    ioctl(usb_fd, FIONREAD, &bytes_available);
    logger.debug("tooMuchBytesAvailable") << bytes_available;
    if(bytes_available > sizeof(sense_link::Message)) {
        tcflush(usb_fd, TCIFLUSH);
        logger.error("tooMuchBytesAvailable") << "FLUSHED BUFFER, TOO MUCH BYTES AVAILABLE!";
        ioctl(usb_fd, FIONREAD, &bytes_available);
        logger.debug("tooMuchBytesAvailable") << bytes_available;
        usleep(10);
        return true;
    }
    usleep(10);
    return false;
}

bool UsbSenseLink::initUSB(){
    logger.warn("initUSB") << "new init";
    usb_fd = open(path.c_str(), O_RDWR);
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
    usb_tio.c_cc[VTIME] = 0; //0

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
    sleep(5);

    //tcflush(usb_fd,TCIOFLUSH);

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

bool UsbSenseLink::readMessage(sense_link::Message *message) {
    std::uint8_t messageLength;
    char buffer[255];

    bool timeout = false;
    int bytesDecoded = 0;

    if(!tooMuchBytesAvailable()){
        timeout |= ! readFull((char*)&messageLength, 1);
        timeout |= ! readFull(buffer, messageLength);
        if (timeout) {
            logger.perror("readMessage");
        } else {
            bytesDecoded = sense_link::decodeMessage(message, buffer);

            if(bytesDecoded == -1) {
                logger.warn("readMessage") << "Wrong checksum";
            }

            logger.info("readMessage") << "Read finished with " << bytesDecoded << " bytes";
        }
    }
    if(timeout || tooMuchBytesAvailable()) {
        /// Überprüft oft es einen Input/output Fehler gibt
        /// Wenn ja --> versuche den USB neu zu initialisieren
        is_valid_fd(usb_fd);
        return false;
    }

    // check for wrong checksum
    return bytesDecoded != -1;
}

bool UsbSenseLink::writeMessage(const sense_link::Message *message) {
    char buffer[255];
    std::uint8_t messageLength = sense_link::encodeMessage(message, buffer + 1);
    buffer[0] = messageLength;

    bool timeout = false;
    if(!tooMuchBytesAvailable()){
        timeout = ! writeFull(buffer, messageLength + 1);
        if(timeout){
            logger.perror("writeMessage");
        } else {
            logger.info("writeMessage") << "Send finished with "
                                        << (int)messageLength << " bytes";
        }
    }
    if(timeout || tooMuchBytesAvailable()) {
        /// Überprüft oft es einen Input/output Fehler gibt
        /// Wenn ja --> versuche den USB neu zu initialisieren
        is_valid_fd(usb_fd);
        return false;
    }

    return true;
}

bool UsbSenseLink::cycle(){
    //usleep(10000);
    sense_link::Message m;

    //send MotorVelocity Message
    m.mType = sense_link::SENSOR_DATA;
    m.sType = sense_link::MOTOR_VELOCITY;
    m.id = 1;
    senseBoard->getSensor(sense_link::MOTOR_VELOCITY,1,m.sensorData);

    writeMessage(&m);
    logger.info("cycle") << "Send finished:" << " : " << m.sensorData.MotorVelocity.acceleration;

    sense_link::Message in;
    readMessage(&in);
    logger.info("cycle") << "Read finished:" << m.mType << " sT: " << m.sType;


    sense_link::Message fServo;
    fServo.mType = sense_link::SENSOR_DATA;
    fServo.sType = sense_link::SERVO;
    fServo.id = 1;
    senseBoard->getSensor(sense_link::SERVO,1,fServo.sensorData);
    writeMessage(&fServo);
    logger.info("cycle") << "Send finished:" << m.sensorData.Servo.angle;


    readMessage(&in);
    logger.info("cycle") << "Read finished:" << m.mType << " sT: " << m.sType;

    return true;
}
