//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#include "GuitarBot.h"
#include "Midi_Parser.h"


GuitarBot::GuitarBot(int iMidiChannel, MIDI_Reader& m)
        : EventCallback(iMidiChannel), m_bInitialized(false), m_rtpMidi(*this, RTPMIDI_PORT) {
    guitarTracks.push_back(new GuitarTrack(m));
    currentTrack = guitarTracks[0];
}

GuitarBot::GuitarBot(int iMidiChannel, vector<MIDI_Reader>& m)
        : EventCallback(iMidiChannel), m_bInitialized(false), m_rtpMidi(*this, RTPMIDI_PORT) {
    for(MIDI_Reader& midi : m) guitarTracks.push_back(new GuitarTrack(midi));
    currentTrack = guitarTracks[0];
}

GuitarBot::~GuitarBot()
{
    for(GuitarTrack* gTrack : guitarTracks) delete gTrack;
}


Error_t GuitarBot::init(const std::string &robotName, const std::string& host) {

    m_rightHand.setMoveParameter(getMoveParam(currentTrack, 0));    // 50 manually given as velocity for first movement



    auto err = m_rightHand.init(host);
    if (err != kNoError) {

        return err;
    }

    if (m_rtpMidi.init(robotName) != 0) {
        return kNotInitializedError;
    }

    m_bInitialized = true;
    return kNoError;
}

Error_t GuitarBot::startListening(double timeout_sec) {
    if (!m_bInitialized)
        return kNotInitializedError;

    Error_t err = m_rightHand.start(timeout_sec);
    if (err != kNoError) {
        return err;
    }

    m_pRtpThread = std::make_unique<std::thread>(&GuitarBot::rtpThreadHandler, this);

    if (timeout_sec > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(int(timeout_sec*1e6)));
        stop();
    }
    return kNoError;
}

void GuitarBot::stop() {
    m_rightHand.stop();
    m_rtpMidi.close();
    if (m_pRtpThread)
        if (m_pRtpThread->joinable()) {
            m_pRtpThread->join();
        }
}

Error_t GuitarBot::play(uint8_t uiMidiNote, uint8_t uiMidiVelocity, GuitarBot::Technique technique) {
    std::cout << "Note " << (int)uiMidiNote << "\t velocity " << (int)uiMidiVelocity << std::endl;
    return kNoError;
}

void GuitarBot::setTempo(float bpm)
{
    currentTrack->setTempo(bpm);
}

void GuitarBot::addTrack(MIDI_Reader& m)
{
    guitarTracks.push_back(new GuitarTrack(m));
}

void GuitarBot::peerConnected(int port, const std::shared_ptr<rtppeer>& peer) {
    std::cout << "Peer connected: " << peer->local_name << std::endl;
}

void GuitarBot::peerDisconnected(disconnect_reason_e reason) {
    std::cout << "Peer disconnected. Reason: " << reason << std::endl;
}

void GuitarBot::noteOff(uint8_t velocity, uint8_t note) {
    if (note == 63) {
        m_rightHand.readDepthStatus(64);
        m_rightHand.setMotionStat_d(PandaArm::Running);
        m_rightHand.stop_motor();


    }
}

void GuitarBot::noteOn(uint8_t velocity, uint8_t note) {
    if (note != 63) {

            std::cout << "Note " << (int) note << "\t velocity " << (int) velocity << std::endl;
            //mPandaArm.motion_destroyer = 1.0;

            if (m_index < currentTrack->getChords().size()) {
                m_rightHand.setMoveParameter(getMoveParam(currentTrack, m_index));
                LOG_INFO("MOTION START");

                m_rightHand.desiredMidiVelocity(velocity);
                m_rightHand.setMotionStat_p(PandaArm::Running);
                //add m.parameter.method();
                m_index += 1;
            }

            // track index +1
//            if (m_index >= currentTrack->getChords().size() && track_index < (guitarTracks.size() - 1)) {
//                m_index = 0;
//                track_index++; // now loop track
//                currentTrack = guitarTracks[track_index];
//                m_rightHand.setTrackStat(PandaArm::Next);
//            }

            // track init
            if (m_index == 0) {
                m_rightHand.setTrackStat(PandaArm::Next);
            }

            std::cout << "TRACK: " << track_index << ", " << "INDEX: " << m_index << std::endl;
    }


    // activate motor
    else {
            // m_rightHand.setMotorStatus(true);  // turn on tremolo
            // m_rightHand.setMotorPosition(true);
        m_rightHand.readDepthStatus(82);
        m_rightHand.setMotionStat_d(PandaArm::Running);
        m_rightHand.start_motor(100);
        }
}



// called "after touch" in max
void GuitarBot::channelPressure(uint8_t pressure) {
    m_rightHand.readMidiVelocity(pressure);
    LOG_INFO(pressure);


}

void GuitarBot::cc(uint8_t value, uint8_t key) {
    // timber
    if (key == 7){
            m_rightHand.readTimbreStatus(value);
            m_rightHand.setMotionStat_t(PandaArm::Running);
    }

    // depth
    if (key == 2) {
        m_rightHand.readDepthStatus(value);
        m_rightHand.setMotionStat_d(PandaArm::Running);

    }

    // pick stiffness
    if (key == 4) {
        m_rightHand.setPickStiffness(value);
         LOG_INFO("pick stiffness");
         LOG_INFO(value);

    }

    if (key==3)
    {
        LOG_INFO("program change");
        LOG_INFO(value);

        track_index = value;
        m_index = 0;
        currentTrack = guitarTracks[track_index];
        cout << currentTrack->getChords()[0]->toString() << endl;
        m_rightHand.setMoveParameter(getMoveParam(currentTrack, m_index));
        m_rightHand.setTrackStat(PandaArm::Next);

    }

    // tempo change
    if (key == 5) {

        tempo_temp = value+43;
        currentTrack->setTempo(tempo_temp);
        LOG_INFO(tempo_temp);

    }

    // set motor frequency
    if (key == 6) {
        m_rightHand.setOmega(4+pow(1.00023,pow(value,2)));
    }
}


// for switching track
void GuitarBot::programChange(uint8_t progchange) {

//    LOG_INFO("program change");
//    LOG_INFO(progchange);
//
//        track_index = progchange;
//        m_index = 0;
//        currentTrack = guitarTracks[track_index];
//        cout << currentTrack->getChords()[0]->toString() << endl;
//        m_rightHand.setMoveParameter(getMoveParam(currentTrack, m_index));
//        m_rightHand.setTrackStat(PandaArm::Next);


}

void GuitarBot::rtpThreadHandler() {
    m_rtpMidi.run();
}

GuitarTrack& GuitarBot::getCurrentGuitarTrack()
{
    return *currentTrack;
}


