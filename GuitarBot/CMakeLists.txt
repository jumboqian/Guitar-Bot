find_library(FRANKA_LIBRARY franka REQUIRED hint /usr/local/lib)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(
        ${PROJECT_NAME}
        src/main.cpp
        src/PandaArm.cpp
        src/GuitarBot.cpp
        src/MotionGenerator.cpp
        src/RightHandController.cpp
        src/MIDI_Reader.cpp
        src/GuitarTrack.cpp
        src/GuitarEvent.cpp
        src/GTab.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${FRANKA_LIBRARY} -lpthread -lEposCmd Eigen3::Eigen ${RTPMIDI_LIB} ${MOTOR_CONTROLLER_LIB})
