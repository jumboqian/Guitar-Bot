//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#ifndef GUITARBOT_GUITARBOT_H
#define GUITARBOT_GUITARBOT_H

#include "iostream"
#include "string"

#include "RightHandController.h"
#include "MotorController.h"
#include "rtpmidi.h"
#include "EventCallback.h"
#include "PandaArm.h"
#include "ErrorDef.h"

#include "GBotFret.h"

class GuitarBot: EventCallback {
public:
    enum Technique {
        Pick,
        Strum,
        kNumTechniques
    };
    explicit GuitarBot(int iMidiChannel, MIDI_Reader& m);
    explicit GuitarBot(int iMidiChannel, vector<MIDI_Reader>& m);
    ~GuitarBot();
    Error_t init(const std::string& robotName, const std::string& host);
    Error_t startListening(double timeout_sec=10);
    void stop();
    Error_t play(uint8_t uiMidiNote, uint8_t uiMidiVelocity, Technique technique);

    void setTempo(float bpm);
    void addTrack(MIDI_Reader& m);

    GuitarTrack& getCurrentGuitarTrack();

private:
    void cc(uint8_t value,uint8_t key) override;
    void noteOn(uint8_t velocity, uint8_t note) override;
    void noteOff(uint8_t velocity, uint8_t note) override;
    void channelPressure(uint8_t pressure) override;
    void programChange(uint8_t progchange) override;
    void peerConnected(int port, const std::shared_ptr<rtppeer>& peer) override;
    void peerDisconnected(disconnect_reason_e reason) override;

    void rtpThreadHandler();

    bool m_bInitialized;
    RightHandController m_rightHand;

    //CSVParser *m_pcsvparser;
    RtpMidi m_rtpMidi;
    int m_index = 0;
    int track_index = 0;
    int tempo_temp = 60;
    std::unique_ptr<std::thread> m_pRtpThread;

    vector<GuitarTrack*> guitarTracks;
    GuitarTrack* currentTrack;
};

#endif //GUITARBOT_GUITARBOT_H
