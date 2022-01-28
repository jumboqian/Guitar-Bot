//
// Created by codmusic on 5/6/21.
// Modified by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#ifndef GUITARBOT_CPP_DEFINES_H
#define GUITARBOT_CPP_DEFINES_H

#include "pch.h"

//CSV parser
#define FILE_PATH "/home/codmusic/PycharmProjects/pythonProject2/timbre_testing.csv"

//RTPMIDI
#define RTPMIDI_PORT 5004
#define MIDI_CHANNEL 2

//PANDA ARM
#define NUM_PANDA_JOINTS 7
#define ROBOT_NAME "Guitar Bot"
#define HOST "130.207.59.47"
static const inline double kStringSpacing = 0.0140;   ///kStringSpacing = 0.0150
static const inline double kBridgeHeight = 0.01;
static const inline int Error_threshold = 5;
// static const inline double kMinTime = 0.3 / 7.0;

//MAXON MOTOR
#define DEFAULT_MIDI_VELOCITY 80u
#define TARGET_POSITION -100


struct config_t {
    std::string strModelFilePath;
    std::string strMappingFilePath;
    uint8_t iNumVotes;
    float fRMSThreshold;
    uint16_t usPort;
    unsigned int iHopLength;
    unsigned int iWindowLength;
};

#endif //GUITARBOT_CPP_DEFINES_H