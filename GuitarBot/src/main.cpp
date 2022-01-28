//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#include <iostream>
#include "Defines.h"
#include "GuitarBot.h"
#include "GBotFret.h"
#include <csignal>

GuitarBot *pbot = nullptr;

void StopHandler(int signum) {
    if (pbot){
        pbot->stop();
        delete pbot;
    }
}

int main(int argc, char* argv[]) {
    vector<MIDI_Reader> midis = MIDI_Reader::readFolder("/home/codmusic/Downloads/Midis");
//    MIDI_Reader m("/home/codmusic/Downloads/playtime_limit.mid");
// GuitarTrack gTrack(midis);
// std::cout << gTrack.toString() << endl;


    signal(SIGINT, StopHandler);
    pbot = new GuitarBot(MIDI_CHANNEL, midis);


//    pbot->setTempo(20);
    auto err = pbot->init(ROBOT_NAME, HOST);
    if (err != kNoError){
        delete pbot;
        return err;
    }
    err = pbot->startListening(1000);
    delete pbot;
    return err;
}

