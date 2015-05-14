#include <string>
#include <cmath>

#include <usb_sense_link.h>
#include "lms/datamanager.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <termios.h>

#ifndef __APPLE__
  #include <linux/usbdevice_fs.h>
#endif

#include <algorithm>

const int UsbSenseLink::MAX_LOOP_COUNT = 1000;

bool UsbSenseLink::initialize(){
    config = getConfig();
    path = config->get<std::string>("path");
    std::vector<std::string> c = config->getArray<std::string>("channels");

    // get all channels with name and sensors from config
    for(uint i = 0; i < c.size(); i++){
        channels[i].name = c[i];
        std::vector<int> s = config->getArray<int>(channels[i].name);
        for(uint j = 0; j < channels[i].sensor.size(); j++){
            channels[i].sensor[j] = static_cast<sense_link::SensorType> (s[j]);
        }
    }
    
    // open sense board data channels
    for(uint i = 0; i < channels.size(); i++){
        senseBoard[i] = datamanager()
                ->writeChannel<sense_link::SenseBoard>(this, channels[i].name);
    }
    
    logger.info("device") << "Opening USB device at " << path;
    
    if(!initUSB())
    {
        return false;
    }
    
    // Start receiver thread
    receiverThread = std::thread(&UsbSenseLink::receiver, this);
    
    return true;
}

bool UsbSenseLink::is_valid_fd(int fd) {
#ifdef __APPLE__
    // TODO
    int rc = 0;
#else
    /// Sende Anfrage damit ioctl errno neu setzt
    int rc = ioctl(fd, USBDEVFS_CONNECTINFO, 0);
#endif

    /// Wenn rc != 0 gab es einen Fehler
    if (rc == 0){
        return true;
    }

    logger.perror("is_valid_fd") << "Error in ioctl";
    /// Haben wir einen Input/Output error? -> usb neu initialisieren
    if(errno == EIO) {
        close(fd);
        while(!initUSB()) {
            usleep(100);
        }
    }

    return false;
}

bool UsbSenseLink::initUSB(){
    logger.warn("initUSB") << "new init";
    usb_fd = open(path.c_str(), O_RDWR | O_NDELAY);
    if (usb_fd < 0) {
        logger.perror("init") << "Open Senseboard";
        return false;
    }
    
    // Set configuration
    setUSBConfig(usb_fd);
    
    // Set blocking I/O
    setBlocking( usb_fd, true );

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
    sleep( config->get<unsigned int>("init_sleep", 0) );

    //tcflush(usb_fd,TCIOFLUSH);

    return true;
}

bool UsbSenseLink::setUSBConfig(int fd)
{
    struct termios tio;
    
    ///Termios
    //further reading http://en.wikibooks.org/wiki/Serial_Programming/termios
    tcgetattr(fd, &tio);

    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // tio.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    tio.c_oflag = 0;
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    tio.c_cflag &= ~(CSIZE | PARENB);
    tio.c_cflag |= CS8;
    
    // Ignore modem control lines
    tio.c_cflag |= (CLOCAL | CREAD | CRTSCTS);
    
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0; //0

    // first set speed and then set attr!!!
    if(cfsetispeed(&tio, B115200) < 0 || cfsetospeed(&tio, B115200) < 0) {
        logger.perror("init") << "Baud rate";
    }

    if(tcsetattr(fd, TCSANOW, &tio) < 0) {
        logger.perror("init") << "SET ATTR";
    }
    
    return true;
}

bool UsbSenseLink::deinitUSB()
{
    if( usb_fd >= 0 )
    {
        close(usb_fd);
    }
    
    usb_fd = -1;
    
    return true;
}

bool UsbSenseLink::setBlocking( int fd, bool blocking )
{
    int flags = fcntl(fd, F_GETFL);
    if( blocking )
    {
        // Set to blocking
        flags = flags & ( ~O_NONBLOCK );
    } else {
        // Set to non-blocking
        flags = flags | O_NONBLOCK;
    }
    
    auto result = fcntl(fd, F_SETFL, flags);
    
    if(result == -1)
    {
        logger.perror("setBlocking");
        return false;
    }
    return true;
}

bool UsbSenseLink::deinitialize(){
    logger.info("deinitialize") << "Close Senseboard";

    deinitUSB();
    
    // Wait for receiver thread to be finished with current cycle
    receiverThread.join();

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

    timeout |= ! readFull((char*)&messageLength, 1);
    timeout |= ! readFull(buffer, messageLength);
    if (timeout) {
        logger.perror("readMessage");
        is_valid_fd(usb_fd);
        return false;
    } else {
        bytesDecoded = sense_link::decodeMessage(message, buffer);

        if(bytesDecoded == -1) {
            logger.warn("readMessage") << "Wrong checksum";
        }

        logger.debug("readMessage") << "Read finished with " << bytesDecoded << " bytes";
    }

    // check for wrong checksum
    return bytesDecoded != -1;
}

bool UsbSenseLink::writeMessage(const sense_link::Message *message) {
    char buffer[255];
    std::uint8_t messageLength = sense_link::encodeMessage(message, buffer + 1);
    buffer[0] = messageLength;

    bool timeout = false;
    timeout = ! writeFull(buffer, messageLength + 1);
    if(timeout){
        logger.perror("writeMessage");
        is_valid_fd(usb_fd);
        return false;
    } else {
        logger.debug("writeMessage") << "Send finished with "
                                     << (int)messageLength << " bytes";
    }

    return true;
}

void UsbSenseLink::receiver()
{
    sense_link::Message m;
    
    while( usb_fd >= 0 )
    {
        if(readMessage(&m))
        {
            // TODO
        }
        else
        {
            // recv_errors++;
            logger.error("receiverThread") << "Error reading message";
        }
    }
    
    logger.warn("receiverThread") << "Terminating receiver thread";
}

bool UsbSenseLink::cycle(){

    sense_link::Message in;
    readMessage(&in);
    //logger.info("cycle") << "Read finished:" << in.mType << " sT: " << in.sensor;


    // iteriere über jeden Datenkanal senseBoard
    for(uint i = 0; i < senseBoard.size(); i++){

        // check if sensorType is in Datachannel senseBoard[i]
        if(std::find(channels[i].sensor.begin(), channels[i].sensor.end(), in.sensor)!=channels[i].sensor.end()){

            // set SensorType
            senseBoard[i]->setSensor(in.sensor, in.id, in.sensorData);


        } // end all Datachannels
    } // end loop


    sense_link::Message out;
    // iteriere über jeden Datenkanal senseBoard
    for(uint i = 0; i < senseBoard.size(); i++){

            // TODO
            // set SensorType
            senseBoard[i]->getActuator(out.actuator, out.id, out.actuatorData);
            writeMessage(&out);

    } // end loop




    return true;
}
