//
// Created by Raghavasimhan Sankaranarayanan on 4/14/21.
//

#include "rtpmidi.h"

RtpMidi::RtpMidi(EventCallback& callback,
                 int port) :
                 m_callback(callback),
                 m_iPort(port),
                 m_server("forest_server", to_string(m_iPort)),
                 m_pAvahiClient(nullptr),
                 m_pPoll(nullptr),
                 m_bInitialized(false)
{
    m_server.connected_event.connect([this](const std::shared_ptr<::rtpmidid::rtppeer>& peer) {
        peer->midi_event.connect([this](rtpmidid::io_bytes_reader data){callbackHandler(data);});
        peer->disconnect_event.connect([this](auto reason) { m_callback.peerDisconnected(reason); });
        m_callback.peerConnected(m_iPort, peer);
    });
}

RtpMidi::~RtpMidi() {
    if (m_pAvahiClient)
        avahi_client_free(m_pAvahiClient);
    if (m_pPoll)
        avahi_simple_poll_free(m_pPoll);
    avahi_free(m_pAvahiClient);

    rtpmidid::poller.close();
}

int RtpMidi::init(const std::string& robotName) {
    /* Allocate main loop object */
    if (!(m_pPoll = avahi_simple_poll_new())) {
        fprintf(stderr, "Failed to create simple poll object.\n");
        return 1;
    }

    name = avahi_strdup(robotName.c_str());

    int error;
    m_pAvahiClient = avahi_client_new(avahi_simple_poll_get(m_pPoll), (AvahiClientFlags)0, client_callback, nullptr, &error);
    if (!m_pAvahiClient) {
        fprintf(stderr, "Failed to create client: %s\n", avahi_strerror(error));
        return 2;
    }

    m_bInitialized = true;
    return 0;
}

int RtpMidi::run() const {
    if (!m_bInitialized)
        return 1;
    try {
        while (rtpmidid::poller.is_open()) {
            rtpmidid::poller.wait();
        }
    } catch (const std::exception &e) {
        ERROR("{}", e.what());
        return 1;
    }

    return 0;
}

void RtpMidi::callbackHandler(rtpmidid::io_bytes_reader &data) {
    uint8_t current_command = 0;
    while (data.position < data.end) {
        int maybe_next_command = data.read_uint8();
        if (maybe_next_command & 0x80) {
            current_command = maybe_next_command;
        } else {
            --data.position;
        }

        auto type = current_command & 0xF0;
        auto ch = (current_command & 0x0F) + 1;
        if (type != 0xF0)
            if (!m_callback.canParseMessage(ch)) {
                std::cout << "Not parsing. Channel: " << (int)ch << std::endl;
                return;
            }

        switch (type) {
            case 0xB0:
                m_callback.cc(data.read_uint8(), data.read_uint8());
                break;
            case 0x90:
                m_callback.noteOn(data.read_uint8(), data.read_uint8());
                break;
            case 0x80:
                m_callback.noteOff(data.read_uint8(), data.read_uint8());
                break;
            case 0xC0:
                m_callback.programChange(data.read_uint8());
                break;
            case 0xD0:
                m_callback.channelPressure(data.read_uint8());
                break;
            case 0xF0: {
                switch (current_command) {
                    case 0xFA:
                        m_callback.clockStart();
                        break;
                    case 0xFB:
                        m_callback.clockContinue();
                        break;
                    case 0xFC:
                        m_callback.clockStop();
                        break;
                    case 0xF8:
                        m_callback.clock();
                        break;
                    case 0xF2: {
                        auto lsb = data.read_uint8();
                        auto msb = data.read_uint8();
                        auto position = (msb << 7) + lsb;
                        m_callback.songPositionPointer(position);
                    } break;
                    default:
                        WARNING("MIDI sys command {:02X} not implemented yet", current_command);
                        break;
                }
            }
                break;
            default:
                WARNING("MIDI command type {:02X} not implemented yet", type);
                break;
        }

        if (data.position < data.end)
            data.read_uint8();
    }
}

void RtpMidi::client_callback(AvahiClient *c, AvahiClientState state, void *userdata) {
    assert(c);
    /* Called whenever the client or server state changes */
    switch (state) {
        case AVAHI_CLIENT_S_RUNNING:
            /* The server has startup successfully and registered its host
             * name on the network, so it's time to create our services */
            create_services(c);
            break;
        case AVAHI_CLIENT_FAILURE:
            fprintf(stderr, "Client failure: %s\n", avahi_strerror(avahi_client_errno(c)));
            avahi_simple_poll_quit(simple_poll);
            break;
        case AVAHI_CLIENT_S_COLLISION:
            /* Let's drop our registered services. When the server is back
             * in AVAHI_SERVER_RUNNING state we will register them
             * again with the new host name. */
        case AVAHI_CLIENT_S_REGISTERING:
            /* The server records are now being established. This
             * might be caused by a host name change. We need to wait
             * for our own records to register until the host name is
             * properly established. */
            if (group)
                avahi_entry_group_reset(group);
            break;
        case AVAHI_CLIENT_CONNECTING:
            break;
    }
}

void RtpMidi::close() {
    rtpmidid::poller.close();
    m_bInitialized = false;
}
