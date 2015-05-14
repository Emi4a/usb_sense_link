#include <sensors.h>
#include <lms/module.h>
#include "sense_board.h"
#include <thread>

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
     * Receiver task executed in background as ReceiverThread
     */
    void receiver();

    std::string path;
    const lms::type::ModuleConfig* config;
    int usb_fd;
    
    struct termios usb_tio;
    std::vector<sense_link::SenseBoard*> senseBoard;

    struct Channel {
        std::string name;
        std::vector<sense_link::SensorType> sensor;
    };

    std::vector<Channel> channels;
    
    //! Receiver thread reading sensor data in the background
    std::thread receiverThread;
    
};
