#include <lms/module.h>
#include <sense_link/sensors.h>
#include <sense_link/actuators.h>
#include <thread>
#include <mutex>

class UsbSenseLink:public lms::Module {
public:
    bool initialize();
    bool deinitialize();
    bool cycle();
private:
    /**
     * @brief Maximum loop iterations for readFull()
     * and writeFull().
     */
    static const int MAX_LOOP_COUNT;

    /**
     * @brief Read the full given number of bytes
     * into the buffer.
     *
     * The function will ignore errors from the read()
     * function.
     *
     * @param buffer beginning of a buffer
     * @param bufSize size of the buffer
     * @return true if succesful, false if loop count
     * was reached
     */
    bool readFull(char* buffer, int bufSize);

    /**
     * @brief Write the full given number of bytes
     * into the stream.
     *
     * @param buffer beginnging of a buffer
     * @param bufSize size of the buffer
     * @return true if successful, false if loop count
     * was reached
     */
    bool writeFull(const char* buffer, int bufSize);

    bool readMessage(sense_link::Message *message);

    bool writeMessage(const sense_link::Message *message);

    bool initUSB();
    bool deinitUSB();
    bool setUSBConfig(int fd);
    
    bool is_valid_fd(int fd);
    
    /**
     * Set file descriptor to blocking I/O mode
     * @param fd The usb device descriptor
     * @param blocking Whether to block (true) or not (false)
     */
    bool setBlocking( int fd, bool blocking );
    
    /**
     * Read and increase next sending sequence id
     *
     * @return The sequence identifier
     */
    uint8_t getSequence();
    
    /**
     * Check received sequence id (for packet loss)
     *
     * @param sequence The received sequence id
     * @return The number of messages lost/skipped
     */
    uint8_t checkSequence( uint8_t sequence );
    
    /**
     * Receiver task executed in background as ReceiverThread
     */
    void receiver();

    //! Path to USB device
    std::string path;
    
    //! USB device handle
    int usb_fd;
    
    //! Module config
    const lms::type::ModuleConfig* config;
    
    //! Temporary buffer for incomming sensor data
    sense_link::Sensors receivingSensors;
    std::mutex receivingSensorsMutex;
    
    //! List of open actuator channels
    std::map<std::string, sense_link::Actuators* > actuatorChannels;
    
    //! List of open sensor channels
    std::map<std::string, sense_link::Sensors*> sensorChannels;
    
    //! Map specifying a list of channels (by name) for each sensor
    std::map< sense_link::SensorType, std::set<std::string> > sensorChannelsMap;
    
    //! Receiver thread reading sensor data in the background
    std::thread receiverThread;
    
    //! Sending sequence counter (the next to-be-send sequence id)
    uint8_t sendingSequence;
    
    //! Receiving sequence counter (the latest received sequence id)
    uint8_t receivingSequence;
};
