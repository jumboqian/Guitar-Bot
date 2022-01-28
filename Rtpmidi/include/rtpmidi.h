//
// Created by Raghavasimhan Sankaranarayanan on 4/14/21.
//

#ifndef RTPMIDIPEEREXAMPLE_RTPMIDI_H
#define RTPMIDIPEEREXAMPLE_RTPMIDI_H

#include <iostream>
#include <string>
#include <functional>

#include "rtpmidid/rtppeer.hpp"
#include "rtpmidid/iobytes.hpp"
#include <rtpmidid/rtpserver.hpp>
#include "rtpmidid/poller.hpp"

#include "EventCallback.h"
#include "AvahiClient.h"

using namespace std;
typedef rtpmidid::rtppeer rtppeer;
typedef rtpmidid::rtppeer::disconnect_reason_e disconnect_reason_e;

class RtpMidi {
public:
    explicit RtpMidi(EventCallback& callback, int port=5004);
    int init(const std::string& robotName);

    ~RtpMidi();

    int run() const;

    void close();

    void callbackHandler(rtpmidid::io_bytes_reader &data);
    static void client_callback(AvahiClient *c, AvahiClientState state, AVAHI_GCC_UNUSED void * userdata);

private:
    int m_iPort;
    EventCallback& m_callback;
    rtpmidid::rtpserver m_server;

    AvahiClient *m_pAvahiClient;
    AvahiSimplePoll* m_pPoll;
    bool m_bInitialized;
};


#endif //RTPMIDIPEEREXAMPLE_RTPMIDI_H
