#ifndef GUITAR_EVENT_H
#define GUITAR_EVENT_H

#include <vector>
#include <string>

// Event Types

#define TEMPO 1
#define NOTE 2
#define CHORD 3

// Fret Restrictions

#define MAXFRET 19
#define FRET_RANGE 3

// Base class
class GuitarEvent
{
protected:

	friend class GuitarTrack;

	int tick;
	char type;

public:

	GuitarEvent(int, char);
	virtual ~GuitarEvent();

	void setTick(int);

	int getTick() const;
	char getType() const;

	virtual std::string toString() = 0;
};

// TempoEvent changes the play_time of a quarter note in microseconds
class TempoEvent : public GuitarEvent
{
private:

	int tempo_us;

public:

	TempoEvent(int, int);
	~TempoEvent();

	int getTempoMicroseconds() const;

	std::string toString();
};

// NoteEvent
class NoteEvent : public GuitarEvent
{
private:

	int channel;
	int endTick;
	int note;
	int velocity;
	char g_string;
	char fret;

public:

	NoteEvent(int, int, int, int);
	~NoteEvent();

	int getChannel();
	int getDuration();
	int getEndTick();
	int getNote();
	char getGuitarString();
	char getFret();

	void setDuration(int);
	void setEndTick(int t);
	void setGuitarString(int);
	void setFret(char);
	void setFret(unsigned char*);

	std::vector<char> getPossibleFrets(unsigned char*);

	std::string toString();
};

// ChordEvent is a set of NoteEvents
class ChordEvent : public GuitarEvent
{
private:

	bool playable;
	std::vector<NoteEvent> notes;

#define PICK 0
#define STRUM 1
#define TREMOLO 2
#define UP -1
#define DOWN 1

	char direction;

	char contact_string;
	char final_contact_string;

	char condenseStringFret(char, char);
	char extractString(unsigned char);
	char extractFret(unsigned char);

public:

	ChordEvent(int, NoteEvent);
	~ChordEvent();

	void addNote(NoteEvent);

	int getDuration();
	unsigned char getLowestFret();
	unsigned char getHighestFret();

	void sortByPitch();
	void sortByTime();

	void fixDuplicates();

	void setPlayable(bool = 1);
	bool isPlayable();

	bool checkConflict(unsigned char*);

	int checkForNote(int);
	bool removeNote(int);

	void setDirection(char);

	char getTechnique();
	char getDirection();

	char getHighestString();
	char getLowestString();

	void setContactStrings();

	char getContactString();
	char getFinalContactString();
#define PD_P 0.5
#define PD_E 0.5
#define PU_P 0.5
#define PU_E 0.5
#define SD_P 1
#define SD_E 1.5
#define SU_P 1.5
#define SU_E 1

//#define PD_P 1.0
//#define PD_E 0.85
//#define PU_P 0.75
//#define PU_E 1.15
//#define SD_P 1.5
//#define SD_E 1.5
//#define SU_P 1.5
//#define SU_E 1.5

	float getPreparePosition();
	float getFinalPosition();

	std::vector<NoteEvent>& getNotes();
	unsigned int getNumNotes();
	uint64_t notesToLong();

	std::string toString();
};

#endif