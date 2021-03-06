#include "GuitarTrack.h"

#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>

#include <iostream>

GuitarTrack::GuitarTrack(MIDI_Reader& m)
{
	tuning[0] = 40; // E
	tuning[1] = 45; // A
	tuning[2] = 50; // D
	tuning[3] = 55; // G
	tuning[4] = 59; // B
	tuning[5] = 64; // E

	gTab = GTab(tuning);

	processMIDI(m);

	setTempo(120);
	calcTickTime();

	state = READY;
}

GuitarTrack::~GuitarTrack()
{
	for (GuitarEvent* ptr : g_track)
	{
		delete ptr;
	}
}

// Process MIDI data into playable chord events stored in the guitar track.
void GuitarTrack::processMIDI(MIDI_Reader& m_reader)
{
	division = m_reader.getDivision();
	divType = m_reader.getDivType();
	divTick = m_reader.getDivTick();
	nSMPTEFormat = m_reader.getNSMPTEFormat();

	vector<GuitarEvent*> tempNotes; // holds NoteEvents without a play_time
	vector<GuitarEvent*> tempTrack;

	int minimumDuration = 0x7FFFFFFF;

	for (Track_Chunk m_track : m_reader.getTracks())
	{
		int tick = 0;
		unsigned char rs = 0;

		for (MTrk_Event m_event : m_track.getEvents())
		{
			tick += m_event.getDeltaTime();

			int index = 0;

			unsigned char status = m_event.getData(index);

			if (status < 0x80)
			{
				status = rs;
			}
			else
			{
				rs = status;
				index++;
			}

			bool zero_velocity = 0; // Note-on event with 0-velocity is same as note-off event

			if ((status & 0xF0) == 0b10010000) // Note on event
			{
				if (m_event.getData(index + 1))
				{
					GuitarEvent* ptr = new NoteEvent(tick, status & 0x0F, m_event.getData(index), m_event.getData(index + 1));
					tempNotes.push_back(ptr);
					continue;
				}
				zero_velocity = 1;
			}
			if (((status & 0xF0) == 0b10000000) || zero_velocity) // Note off event
			{
				for (unsigned int i = 0; i < tempNotes.size(); i++)
				{
					NoteEvent* n_ptr = (NoteEvent*) tempNotes[i];

					if (n_ptr->getNote() == m_event.getData(index))
					{
						if (n_ptr->getChannel() == (status & 0x0F))
						{
							n_ptr->setEndTick(tick);
							if (n_ptr->getDuration() != 0 && n_ptr->getDuration() < minimumDuration)
							{
								minimumDuration = n_ptr->getDuration();
							}
							tempTrack.push_back(n_ptr);
							tempNotes.erase(tempNotes.begin() + i);
							break;
						}
					}
				}
			}
			else if (status == 0xFF) // Meta-Events
			{
				if (m_event.getData(index) == 0x51 && m_event.getData(index + 1) == 0x03) // Set Tempo
				{
					// Placeholder
				}
			}
		}
	}

	int exponent = round(log2((1.0 * minimumDuration) / divTick));
	int updown_beat_tick = round(divTick * pow(2, exponent));


	ChordEvent* chord_ptr = 0;


	for (unsigned int i = 0; i < tempTrack.size(); i++)
	{

		GuitarEvent* ptr = tempTrack[i];
		if (ptr->getType() == NOTE)
		{
			NoteEvent* n_ptr = (NoteEvent*) ptr;
			if (chord_ptr) // check if a chord already exists
			{
				if (n_ptr->getTick() < chord_ptr->getTick() + chord_ptr->getDuration()) // check if the starting tick of the note overlaps with the chord's play_time
				{
					if (chord_ptr->getNotes().size() >= 6)
					{
						/*
						// Set the strings and frets
						chord_ptr->setFrets(tuning);
						chord_ptr->fixDuplicates();
						*/
						// If the chord has multiple notes, set the strumming direction based on which beat it is on
						if ((chord_ptr->getTick() / updown_beat_tick) % 2)
						{
                            chord_ptr->setDirection(UP);
						}
						else
						{
							chord_ptr->setDirection(DOWN);
						}
						//chord_ptr->setContactStrings();

						// create a new ChordEvent and add it to the track
						chord_ptr = new ChordEvent(n_ptr->getTick(), *n_ptr);
						g_track.push_back(chord_ptr);
						chordEvents.push_back(chord_ptr);
					}
					else
					{
						if (n_ptr->getTick() < chord_ptr->getTick()) chord_ptr->setTick(n_ptr->getTick());
						chord_ptr->addNote(*n_ptr); // add the note to the chord
						continue;
					}
				}
				else
				{
					// Set the strings and frets
					/*
					chord_ptr->setFrets(tuning);
					chord_ptr->fixDuplicates();
					*/
					// If the chord has multiple notes, set the strumming direction based on which beat it is on
					if (chord_ptr->getTechnique() == STRUM)
					{
						if ((chord_ptr->getTick() / updown_beat_tick) % 2)
						{

                            chord_ptr->setDirection(UP);
						}
						else
						{
							chord_ptr->setDirection(DOWN);
						}
						//chord_ptr->setContactStrings();
					}
					// create a new ChordEvent and add it to the track
					chord_ptr = new ChordEvent(n_ptr->getTick(), *n_ptr);
					g_track.push_back(chord_ptr);
					chordEvents.push_back(chord_ptr);
				}
			}
			else
			{
				// create a new ChordEvent and add it to the track
				chord_ptr = new ChordEvent(n_ptr->getTick(), *n_ptr);
				g_track.push_back(chord_ptr);
				chordEvents.push_back(chord_ptr);
			}

			delete ptr;
		}
		else if (ptr->getType() == TEMPO)
		{
			g_track.push_back(ptr);
			tempoChanges.push_back((TempoEvent*) ptr);
		}
		else
		{
			delete ptr;
		}
	}

	gTab.setFrets(chordEvents);

	// Last Chord
	if (chord_ptr)
	{
		/*
		// Set the strings and frets
		chord_ptr->setFrets(tuning);
		chord_ptr->fixDuplicates();
		*/
		// If the chord has multiple notes, set the strumming direction based on which beat it is on
		if (chord_ptr->getTechnique() == STRUM)
		{
			if ((chord_ptr->getTick() / updown_beat_tick) % 2)
			{
				chord_ptr->setDirection(UP);
			}
			else
			{
				chord_ptr->setDirection(DOWN);
			}
			chord_ptr->setContactStrings();
		}
	}

	// Set the picking direction for picked notes

	ChordEvent* last = 0;
	ChordEvent* current = 0;
	ChordEvent* next = 0;

	for (int i = 0; i < chordEvents.size();)
	{
		while (!current)
		{
			current = chordEvents[i];
			i++;
		}
		while (!next && i < chordEvents.size())
		{
			next = chordEvents[i];
			i++;
		}

		// Check if the chord is just a single picked note
		if (current->getTechnique() == PICK)
		{
			// Check conditions for when picking direction should be up
			if	((next && (next->getTechnique() == PICK)) ?
					((next->getNotes()[0].getGuitarString() < current->getNotes()[0].getGuitarString())
					|| ((next->getNotes()[0].getGuitarString() == current->getNotes()[0].getGuitarString())
						&& last && last->getTechnique() == PICK && last->getDirection() == DOWN))
					: (last && last->getTechnique() == PICK && last->getDirection() == DOWN)
				)
			{
				current->setDirection(UP);
			}
			else
			{
				current->setDirection(DOWN);
			}
			current->setContactStrings();
		}
		else
		{
			current->setContactStrings();
		}

		last = current;
		current = next;
		next = 0;
	}
}

// Set functions for tempo

void GuitarTrack::setTempo(float t)
{
	tempo = t;
	tempo_us = 60000000.0 / t;
}

void GuitarTrack::setTempoMicroseconds(unsigned int t_us)
{
	tempo = 60000000.0 / t_us;
	tempo_us = t_us;
}

// Calculates how long each tick is in microseconds
void GuitarTrack::calcTickTime()
{
	tick_us = tempo_us / divTick;
}

vector<ChordEvent*>& GuitarTrack::getChords()
{
	return chordEvents;
}

// Returns pointer to a GuitarEvent
const GuitarEvent* GuitarTrack::getEvent(int index)
{
	if (index < 0 || index >= g_track.size()) return 0;

	return g_track[index];
}

// Returns the time in microseconds when a GuitarEvent executes
long long GuitarTrack::getEventTime(int index)
{
	if (index < 0 || index >= g_track.size()) return -1;

	return 0; // PLACEHOLDER
}

long long GuitarTrack::tick_to_us(int tick)
{
	return 0; // PLACEHOLDER
}

double GuitarTrack::tick_to_seconds(int tick)
{
    return 60.0 * tick / (tempo * divTick);
}

// Runs a guitar track

void GuitarTrack::run_track()
{
	vector<NoteEvent*> noteBuffer;
	vector<GuitarEvent*>::iterator eventIterator = g_track.begin();
	GuitarEvent* event = *eventIterator;

	if (state == SETUP) state = RUNNING;

	while (RUNNING)
	{
		if (event != nullptr && tick >= event->getTick())
		{
			switch (event->getType())
			{
			case CHORD:
			{
				ChordEvent* chordPtr = (ChordEvent*)event;
				/*
				if (chordPtr->getLowestFret() < carFret)
				{
					arduinoInterface->addOrder(OrderTicket(0, MOVE, carFret - chordPtr->getLowestFret()));
				}
				else if (chordPtr->getHighestFret() > carFret + 2)
				{
					arduinoInterface->addOrder(OrderTicket(0, MOVE, chordPtr->getHighestFret() - carFret - 2));
				}

				for (NoteEvent note : chordPtr->getNotes())
				{
					unsigned char pin = ((note.getFret() - carFret) * 6) + note.getGuitarString() + 1;
					arduinoInterface->addOrder(OrderTicket(400, ON, pin));
					noteBuffer.push_back(&note);
				}
				*/
				break;
			}
			case TEMPO:
			{
				// Placeholder

				break;
			}
			}

			//cout << event->toString();

			eventIterator++;
			if (eventIterator == g_track.end())
			{
				event = nullptr;
			}
			else
			{
				event = *eventIterator;
			}
		}

		for (int i = 0; i < noteBuffer.size();)
		{
			NoteEvent note = *(noteBuffer[i]);
			if (tick >= noteBuffer[i]->getEndTick())
			{
				/*
				unsigned char pin = ((note.getFret() - carFret) * 6) + note.getGuitarString() + 1;
				arduinoInterface->addOrder(OrderTicket(0, OFF, pin));
				noteBuffer.erase(noteBuffer.begin() + i);
				*/
			}
			else
			{
				i++;
			}
		}

		if (event == nullptr && (noteBuffer.size() == 0))
		{
			state = STOP;
		}
	}
}

void GuitarTrack::run_track_static(GuitarTrack* track)
{
	track->run_track();
}

void GuitarTrack::run()
{
	if (state != READY) return;

	state = SETUP;

	tick = -1;

	thread t(run_track_static, this);
	t.detach();

	auto start = chrono::high_resolution_clock::now();
	auto elapsed = chrono::high_resolution_clock::now();
	long long time_us = chrono::duration_cast<chrono::microseconds>(elapsed - start).count();
	long time_marker = tick_us;

	while (state == SETUP);

	tick++;

	while (state == RUNNING)
	{
		elapsed = chrono::high_resolution_clock::now();
		time_us = chrono::duration_cast<chrono::microseconds>(elapsed - start).count();
		while (time_us > time_marker)
		{
			tick++;
			time_marker += tick_us;
		}
	}

	state = READY;
}

void GuitarTrack::stop()
{
	if (state != READY) state = STOP;
}

// Creates a string representation of the track
string GuitarTrack::toString()
{
	string str;
	for (GuitarEvent* ptr : g_track)
	{
		str += ptr->toString();
	}
	return str;
}