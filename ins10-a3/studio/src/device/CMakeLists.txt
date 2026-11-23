# ins10device: byte transports (serial port, simulated board) and the command session.
add_library(ins10device STATIC
    boardmodel.cpp boardmodel.h
    calibrationrun.cpp calibrationrun.h
    controller.cpp controller.h
    fakeboard.cpp fakeboard.h
    ports.cpp ports.h
    serial_transport.cpp serial_transport.h
    session.cpp session.h
    simulation.cpp simulation.h
    transport.h
)
target_include_directories(ins10device PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(ins10device PUBLIC ins10core Qt6::Core Qt6::SerialPort Qt6::Concurrent)
ins10_set_warnings(ins10device)
