# ins10core: protocol, recordings, calibration fields and algorithms, time series.
# QtCore and Eigen only: no serial port, no GUI, everything testable headless.
add_library(ins10core STATIC
    attitude.cpp attitude.h
    calib/errors.h
    calib/imu.cpp calib/imu.h
    calib/lsq.cpp calib/lsq.h
    calib/mag.cpp calib/mag.h
    calib/stats.cpp calib/stats.h
    calib/thermal.cpp calib/thermal.h
    calib/workflow.cpp calib/workflow.h
    calibration_fields.cpp calibration_fields.h
    health.cpp health.h
    logfile.cpp logfile.h
    protocol.cpp protocol.h
    recording.cpp recording.h
    scene.cpp scene.h
    series.cpp series.h
    status.cpp status.h
    streamseries.cpp streamseries.h
)
target_include_directories(ins10core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(ins10core PUBLIC Qt6::Core Eigen3::Eigen)
ins10_set_warnings(ins10core)
