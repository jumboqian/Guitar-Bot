add_library(${MOTOR_CONTROLLER_LIB} Src/MotorController.cpp Src/Striker.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)
target_include_directories(
        ${MOTOR_CONTROLLER_LIB} PUBLIC
        /usr/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Include
        ${MOTOR_CONTROLLER_LIB}/Include)

target_link_directories(${MOTOR_CONTROLLER_LIB} PUBLIC /usr/lib)
target_link_libraries(${MOTOR_CONTROLLER_LIB} PUBLIC -lpthread -lEposCmd)

if (ENABLE_MIDI)
    target_include_directories(${MOTOR_CONTROLLER_LIB} PUBLIC ${CMAKE_SOURCE_DIR}/${OSC_LIB}/Include)
    target_link_libraries(${MOTOR_CONTROLLER_LIB} PUBLIC ${OSC_LIB})
endif ()
