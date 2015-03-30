set ( SOURCES 
    "src/usb_sense_link.cpp"
    "src/interface.cpp"
)

set (HEADERS
    "include/usb_sense_link.h"
)

include_directories("include")

add_library ( usb_sense_link MODULE ${SOURCES} ${HEADERS})
target_link_libraries(usb_sense_link PRIVATE lms sense_link)
