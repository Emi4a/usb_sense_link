#include <sensors.h>
#include <lms/module.h>
#include <termios.h>

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

    const lms::type::ModuleConfig* config;
    int usb_fd;
    struct termios usb_tio;
};
