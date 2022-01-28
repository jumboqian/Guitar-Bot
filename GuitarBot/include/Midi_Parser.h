//
// Created by Yilin Zhang on 6/2/21.
//

#pragma once

#include <string>
#include <vector>
#include "GuitarTrack.h"
#include "PandaArm.h"

double mapFunction(double x, double minX, double maxX, double minY, double maxY)
{
    return ((maxY - minY) / (maxX - minX)) * (x - minX) + minY;
}
// PandaArm::MoveParam getMoveParam(GuitarTrack* gTrack, int m_index, uint8_t velocity)
PandaArm::MoveParam getMoveParam(GuitarTrack* gTrack, int m_index) {
#define MOVE_TIME_MINIMUM 0.15  /// 0.23 for 8th note: meaning max bpm is 60 / (0.23*2) = 130
#define PICK_RECOVER_TIME 0.05

    struct PandaArm::MoveParam moveparam{};
    moveparam.m_index = m_index;

    ChordEvent &chord = *gTrack->getChords()[m_index];
    ChordEvent *next = NULL;
    ChordEvent *last = NULL;
    if (m_index + 1 < gTrack->getChords().size()) {
        next = gTrack->getChords()[m_index + 1];
    }
    if(m_index) last = gTrack->getChords()[m_index-1];


    if (chord.getTechnique() == PICK) moveparam.technique = PandaArm::Pick;
    else if (chord.getTechnique() == STRUM) moveparam.technique = PandaArm::Strum;

    if(next && next->getDirection() == DOWN) moveparam.next_direction = 1;
    else moveparam.next_direction = 0;

    moveparam.prepare_position = chord.getPreparePosition();
    moveparam.end_position = chord.getFinalPosition();

    if (next->getTechnique() == PICK) moveparam.next_technique = PandaArm::Pick;
    else if (next->getTechnique() == STRUM) moveparam.next_technique = PandaArm::Strum;


    if (moveparam.prepare_position < moveparam.end_position) moveparam.direction = 1;
    else moveparam.direction = 0;


    cout << "PP: " << moveparam.prepare_position << endl;
    cout << "EP: " << moveparam.end_position << endl;

    // for the last
    if (next == NULL) {
        moveparam.move_step = 0;
        moveparam.play_time = gTrack->tick_to_seconds(chord.getDuration());
        moveparam.prepare_time = 0;
    }
    else {
        if (chord.getTechnique() == STRUM) {         // for STRUM
            moveparam.move_step = next->getPreparePosition() - chord.getFinalPosition();

            std::cout << "MS: " << moveparam.move_step << endl;

            double playstep = abs(chord.getFinalPosition() - chord.getPreparePosition());
            double totalstep = playstep + abs(moveparam.move_step);
            double chord_offset = gTrack->tick_to_seconds(next->getTick() - chord.getTick());

//            LOG_INFO("Chord Offset");
//            LOG_INFO(chord_offset);

            moveparam.play_time =
                    (playstep / totalstep) * (chord_offset);   // longest note in the chord !!!! play_time is move time
            if ( moveparam.play_time > 2.5 )
            {
                moveparam.prepare_time =  2.5;
            }

            moveparam.prepare_time = chord_offset - moveparam.play_time;
            if ( (chord_offset - moveparam.play_time) > 2.0 )
            {
                moveparam.prepare_time =  2.0;
            }

//            if (moveparam.prepare_time >= 0.23) {
//                moveparam.prepare_time = mapFunction(127 - velocity, 0, 126, 0.23, moveparam.prepare_time);
//            } else {
//                moveparam.prepare_time = 0;
//            }
//            LOG_INFO("Play time");
//            LOG_INFO(moveparam.play_time);
//            LOG_INFO("Prepare time");
//            LOG_INFO(moveparam.prepare_time);

        }

        else {             // for PICK
            cout << "pick loop" << endl;
            moveparam.move_step = next->getPreparePosition() - chord.getFinalPosition();

            std::cout << "MS: " << moveparam.move_step << endl;

            double playstep = abs(chord.getFinalPosition() - chord.getPreparePosition());
            double movestep =  abs(moveparam.move_step);
            double chord_offset = gTrack->tick_to_seconds(next->getTick() - chord.getTick());

            if (moveparam.next_direction == moveparam.direction) {
                movestep = movestep + 0.5;

            }

            moveparam.play_time = (playstep / (movestep+playstep)+2) *
                                  (chord_offset);// longest note in the chord !!!! play_time is move time


            if ( moveparam.play_time > MOVE_TIME_MINIMUM )
            {
                moveparam.play_time =  MOVE_TIME_MINIMUM;
            }


            moveparam.prepare_time =  chord_offset - moveparam.play_time;

            if ( (chord_offset - moveparam.play_time) > 1.0 )
            {

                moveparam.prepare_time =  1.0;
            }

            // handling minimum time -- constraint  -- triplet issue unsolved (nathan)
//            if (moveparam.prepare_time < MOVE_TIME_MINIMUM && moveparam.move_step != 0) {
//                moveparam.prepare_time = 0.75;
//            }
//
//            if (moveparam.play_time < MOVE_TIME_MINIMUM) {
//                moveparam.play_time = 3*MOVE_TIME_MINIMUM;
//            }
//            if (moveparam.move_step != 0) {
//                moveparam.prepare_time = 0.75;
//            }
//
//            if (true) {
//                moveparam.play_time = 0.4;
//            }

            std::cout << "moveparam.prepare_time:  " << moveparam.prepare_time << endl;
//            if (moveparam.prepare_time >= 0.23) {
//                moveparam.prepare_time = mapFunction(127 - velocity, 0, 126, 0.23, moveparam.prepare_time);
//            } else {
//                moveparam.prepare_time = 0;
//            }
//
//        }
//
        }
//
//    moveparam.play_time = mapFunction(127 - velocity, 0, 126, 0.23, moveparam.play_time);



        if (moveparam.play_time < MOVE_TIME_MINIMUM ||
            (moveparam.prepare_time < MOVE_TIME_MINIMUM && moveparam.move_step != 0)) {
            LOG_INFO("REEEEE");
            // scream "REEEEEE" really loudly
        }

        //moveparam.prepare_time = get(rowIndex,"prepare_time");  // make it a fixed value
        //moveparam.timbre = get(rowIndex, "timbre");
//        LOG_INFO(moveparam.play_time);
//        LOG_INFO(moveparam.prepare_position);
//        LOG_INFO(moveparam.end_position);
//        LOG_INFO(moveparam.move_step);
        return moveparam;
    }
}


//        // PandaArm::MoveParam getMoveParam(vector<GuitarTrack*>& guitarTracks, int m_index, int track_index, uint8_t velocity)
//PandaArm::MoveParam getMoveParam(vector<GuitarTrack*>& guitarTracks, int m_index, int track_index)
//{
//    #define MOVE_TIME_MINIMUM 0.23  /// for 8th note: meaning max bpm is 60 / (0.23*2) = 130
//    #define PICK_RECOVER_TIME 0.05
//
//    struct PandaArm::MoveParam moveparam{};
//
//    if(track_index > guitarTracks.size()) return moveparam;
//    GuitarTrack* gTrack = guitarTracks[track_index];
//    ChordEvent& chord = *gTrack->getChords()[m_index];
//    ChordEvent* next = NULL;
//    if(m_index + 1 < gTrack->getChords().size())
//    {
//        next = gTrack->getChords()[m_index+1];
//    }
//    else if(track_index + 1 < guitarTracks.size() && m_index + 1 == gTrack->getChords().size()) {
//        next = (guitarTracks[track_index + 1]->getChords())[0];
//    }
//
//
//    if(chord.getTechnique() == PICK) moveparam.technique = PandaArm::Pick;
//    else moveparam.technique = PandaArm::Strum;
//
//    moveparam.prepare_position = chord.getPreparePosition() ;
//    moveparam.end_position = chord.getFinalPosition() ;
//
//    cout << "PP: " << moveparam.prepare_position << endl;
//    cout << "EP: " << moveparam.end_position << endl;
//
//    // for the last
//    if(next == NULL)
//    {
//        moveparam.move_step = 0;
//        moveparam.play_time = gTrack->tick_to_seconds(chord.getDuration());
//        moveparam.prepare_time = 0;
//    }
//    else
//    {
//        if(chord.getTechnique() == STRUM) {         // for STRUM
//            moveparam.move_step = next->getPreparePosition() - chord.getFinalPosition();
//
//            std::cout << "MS: " << moveparam.move_step << endl;
//
//            double playstep = abs(chord.getFinalPosition() - chord.getPreparePosition());
//            double totalstep = playstep + abs(moveparam.move_step);
//            double chord_offset = gTrack->tick_to_seconds(next->getTick() - chord.getTick());
//            if(chord_offset < 0)
//            {
//                gTrack->tick_to_seconds(chord.getDuration() + 1);
//            }
//
//            moveparam.play_time = (playstep / totalstep) * chord_offset;   // longest note in the chord !!!! play_time is move time
//            moveparam.prepare_time = chord_offset - moveparam.play_time;
////            if (moveparam.prepare_time >= 0.23) {
////                moveparam.prepare_time = mapFunction(127, 0, 126, 0.23, moveparam.prepare_time);
////            } else {
////                moveparam.prepare_time = 0;
////            }
//        }
//        else {             // for PICK
//            cout << "pick loop" << endl;
//            moveparam.move_step = next->getPreparePosition() - chord.getFinalPosition();
//
//            std::cout << "MS: " << moveparam.move_step << endl;
//
//            double playstep = abs(chord.getFinalPosition() - chord.getPreparePosition());
//            double totalstep = playstep + abs(moveparam.move_step);
//            double chord_offset = gTrack->tick_to_seconds(next->getTick() - chord.getTick());
//            if(chord_offset < 0)
//            {
//                gTrack->tick_to_seconds(chord.getDuration() + 1);
//            }
//
//            moveparam.play_time = (playstep / totalstep) * (chord_offset-0.1);   // longest note in the chord !!!! play_time is move time
//            moveparam.prepare_time = 0.1 + chord_offset - moveparam.play_time;
//            if (moveparam.prepare_time >= 0.23) {
//                moveparam.prepare_time = mapFunction(127, 0, 126, 0.23, moveparam.prepare_time);
//            } else {
//                moveparam.prepare_time = 0;
//            }
//
//        }
//
//    }
//
//    moveparam.play_time = mapFunction(127, 0, 126, 0.23, moveparam.play_time);
//
//
//
//    if(moveparam.play_time < MOVE_TIME_MINIMUM || (moveparam.prepare_time < MOVE_TIME_MINIMUM && moveparam.move_step != 0))
//    {
//        LOG_INFO("REEEEE");
//        // scream "REEEEEE" really loudly
//    }
//
//    //moveparam.prepare_time = get(rowIndex,"prepare_time");  // make it a fixed value
//    //moveparam.timbre = get(rowIndex, "timbre");
////        LOG_INFO(moveparam.play_time);
////        LOG_INFO(moveparam.prepare_position);
////        LOG_INFO(moveparam.end_position);
////        LOG_INFO(moveparam.move_step);
//    return moveparam;
//}



