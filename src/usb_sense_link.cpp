#include <string>
#include <cmath>
#include <utility>

#include <usb_sense_link.h>
#include <lms/datamanager.h>
#include <lms/extra/time.h>

#include <sense_link/utils.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <termios.h>

#ifndef __APPLE__
  #include <linux/usbdevice_fs.h>
#endif

const int UsbSenseLink::MAX_LOOP_COUNT = 1000;

bool UsbSenseLink::initialize(){
    config = getConfig();
    path = config->get<std::string>("path");
    
    // Init sensor channels
    for( const auto& channel : config->getArray<std::string>("sensorChannels") )
    {
        // Open channel
        logger.info() << "Initializing sensor channel '" << channel << "'";
        sensorChannels[ channel ] = datamanager()->writeChannel<sense_link::Sensors>(this, channel);
        
        for( const auto& sensor : config->getArray<std::string>( channel ) )
        {
            if( sensor == "*" )
            {
                // Wildcard matching: All sensors should be added to this channel
                logger.info(channel) << "Channel is configured as wildcard channel";
                
                std::underlying_type<sense_link::SensorType>::type i = 0;
                auto end = static_cast< std::underlying_type<sense_link::SensorType>::type >( sense_link::SensorType::__END__ );
                for(; i < end; i++ )
                {
                    // Iterate over all possible sensor types
                    auto sensorType = static_cast<sense_link::SensorType>( i );
                    sensorChannelsMap[ sensorType ].insert(channel);
                }
                break;
            }
            
            // Add all sensor -> channel mappings
            logger.info(channel) << "Adding sensor '" << sensor << "'";
            sense_link::SensorType sensorType;
            if(sense_link::Utils::sensorTypeFromName( sensor, sensorType ))
            {
                sensorChannelsMap[ sensorType ].insert(channel);
            }
            else
            {
                logger.error(channel) << "Unknown sensor type '" << sensor << "'";
            }
        }
    }
    
    if( sensorChannels.size() == 0 )
    {
        logger.warn() << "No sensor channels defined!";
    }
    
    // Init actuator channels
    for( const auto& channel : config->getArray<std::string>("actuatorChannels") )
    {
        // Open channel
        logger.info() << "Initializing actuator channel '" << channel << "'";
        actuatorChannels[ channel ] = datamanager()->writeChannel<sense_link::Actuators>(this, channel);
    }
    
    if( actuatorChannels.size() == 0 )
    {
        logger.warn() << "No actuator channels defined!";
    }
    
    logger.info("device") << "Opening USB device at " << path;
    
    if(!initUSB())
    {
        return false;
    }
    
    // Start receiver thread
    shouldStopReceiver = false;
    receiverThread = std::thread(&UsbSenseLink::receiver, this);
    
    return true;
}

bool UsbSenseLink::is_valid_fd(int fd) {
#ifdef __APPLE__
    // Try to read message
    char _buffer = 0;
    int result = read(usb_fd, &_buffer, 1);
    
    if(result == 1) {
        return true;
    }
#else
    /// Sende Anfrage damit ioctl errno neu setzt
    int rc = ioctl(fd, USBDEVFS_CONNECTINFO, 0);
    
    /// Wenn rc != 0 gab es einen Fehler
    if (rc == 0){
        return true;
    }
#endif
    
    logger.perror("is_valid_fd") << "Error in ioctl: Trying to reconnect";
    /// Haben wir einen Input/Output error? -> usb neu initialisieren
    if(errno == EIO || errno == EBADF || errno == ENXIO) {
        close(fd);
        while(!initUSB()) {
            usleep(100);
        }
    }

    return false;
}

bool UsbSenseLink::initUSB(){
    usb_fd = open(path.c_str(), O_RDWR | O_NDELAY);
    if (usb_fd < 0) {
        logger.perror("init") << "Open Senseboard";
        return false;
    }
    
    // Set configuration
    setUSBConfig(usb_fd);
    
    // Set blocking I/O
    setBlocking( usb_fd, true );
    
    // Reset sequence counters
    sendingSequence     = 1; // First sequence id should be 1
    receivingSequence   = 0; // We haven't received anything and are expecting "1" next
    
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

    tcflush(usb_fd,TCIOFLUSH);
    
    logger.info("init") << "Initialized SenseLink connection";

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
    tio.c_cflag |= (CLOCAL | CREAD );
    
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
    
    // Set flag to stop receiver
    shouldStopReceiver = true;
    
    // Wait for receiver thread to be finished with current cycle
    receiverThread.join();
    
    // Deinit interface properly
    deinitUSB();

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
    
    while( usb_fd >= 0 && !shouldStopReceiver )
    {
        if(!readMessage(&m))
        {
            continue;
        }
        
        // Check valid seqence number
        checkSequence( m.sequence );
        
        switch( m.message )
        {
            case sense_link::MessageType::SENSOR_DATA:
                // TODO: convert timestamp
                receivingSensorsMutex.lock();
                receivingSensors.add( m.sensor, m.id, sense_link::SensorMeasurement( m.sensorData, lms::extra::PrecisionTime::now() ) );
                receivingSensorsMutex.unlock();
                break;
            case sense_link::MessageType::ERROR:
                // TODO: error handling
                logger.error("SENSEBOARD_ERROR") << m.error.code;
                break;
            case sense_link::MessageType::TIME:
                // TODO: time syncing
                break;
            default:
                logger.warn("receiver") << "Unexpected message type " << uint32_t( m.message );
                break;
        }
    }
    
    logger.warn("receiverThread") << "Terminating receiver thread";
}

uint8_t UsbSenseLink::getSequence()
{
    return sendingSequence++;
}

uint8_t UsbSenseLink::checkSequence( uint8_t sequence )
{
    // Calculate lost packets
    uint8_t lost = ( sequence - receivingSequence ) - 1;
    
    if( lost > 0 )
    {
        logger.warn("sequence") << "Lost " << uint32_t(lost) << " messages from slave";
    }
    
    // Update sequence counter
    receivingSequence = sequence;
    
    return lost;
}

bool UsbSenseLink::cycle(){

    // Iterate over available actuator data and send
    sense_link::Message msg;
    for( const auto& channel : actuatorChannels )
    {
        for( const auto& data : *(channel.second) )
        {
            msg.message         = sense_link::MessageType::ACTUATOR;
            msg.actuator        = data.first.first;  // Actuator type
            msg.id              = data.first.second; // Actuator id
            msg.sequence        = getSequence();
            msg.actuatorData    = data.second;       // Actuator data payload
            
            writeMessage(&msg);
        }
    }
    
    // Clear actuator data
    for( auto& it : actuatorChannels )
    {
        it.second->clear();
    }
    
    // Clear sensor channels
    for( auto& it : sensorChannels )
    {
        it.second->clear();
    }

    // START Copy temporary sensordata
    receivingSensorsMutex.lock();
    
    // Sort the temp data into the specific channels
    for( const auto& it : receivingSensors )
    {
        auto sensorType = it.first.first;
        auto sensorId   = it.first.second;
        if( sensorChannelsMap.find( sensorType ) == sensorChannelsMap.end() )
        {
            // SensorType not found in any mapping
            logger.warn() << "Received sensor message for SensorType " << uint32_t(sensorType) << " / SensorID " << sensorId;
            continue;
        }
        
        for( const auto& ch : sensorChannelsMap[ sensorType ] )
        {
            for( const auto& data : it.second )
            {
                sensorChannels[ ch ]->add( sensorType, sensorId, data );
            }
        }
    }
    receivingSensors.clear();
    receivingSensorsMutex.unlock();
    // END copy sensordata
    
    return true;
}
