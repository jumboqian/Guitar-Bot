//
// Created by Raghavasimhan Sankaranarayanan on 11/3/20.
//
#include "MotorController.h"

MotorController::MotorController() : m_bIsEnabled(false)
{
    LOG_DEBUG("MotorController Object Created");
}

MotorController::~MotorController()
{
//    std::cout << "Closing striker" << std::endl;
    if (m_bDeviceOpened)
        m_striker.closeDevice();
//    std::cout << "Closed striker" << std::endl;
    LOG_DEBUG("Destroyed MotorController Object");
}

Error_t MotorController::prepareStriker()
{
    Error_t err;
    if ((err = m_striker.openDevice()) != kNoError) {
        LOG_ERROR("Open Device");
        return err;
    }

    m_bDeviceOpened = true;

    if ((err = m_striker.prepare()) != kNoError) {
        LOG_ERROR("Prepare striker");
        return err;
    }

    if ((err = m_striker.setControllerGain(15000000,100000))!= kNoError) {
        LOG_ERROR("Set Gain");
        return err;
    }
    m_bIsEnabled = true;
    LOG_DEBUG("Striker Ready");
    return kNoError;
}

//Error_t MotorController::changeGrip(int grip, bool referGripMap)
//{
//    if (m_gripMap.empty())
//        return kNotInitializedError;
//
//    typedef unsigned long long ull;
//
//    if (!m_bIsEnabled)
//        return kNoError;
//
//    int value = Util::constrain(grip, 0, 127);
//
//    if (referGripMap)
//        value = m_gripMap[grip];
//
//    // Polynomial curve fitting
//    auto PGain = (ull)round(17000000.0 + 53503.078*value + 384.718*pow(value, 2));
//    auto DGain = (ull)round(150000.0 + 325.252*value + 31.539*pow(value, 2));
//
////    std::cout << "Changed Grip " << grip << " - P Gain : " << PGain << "\t D Gain : " << DGain << std::endl;
//
//    if (value > -1)
//        return m_striker.setControllerGain(PGain, DGain);
//
//    return kNoError;
//}




void MotorController::setGripMap(const std::vector<int>& gripMap)
{
    m_gripMap = gripMap;
}

Error_t MotorController::strike(int velocity)
{
    return m_striker.strike(Util::constrain(velocity, 0, 127));
}

void MotorController::setEnabled(bool bEnable)
{
    m_bIsEnabled = bEnable;
}


Error_t MotorController::changeGrip(uint8_t velocity)
{

    typedef unsigned long long ull;

    if (!m_bIsEnabled)
        return kNotInitializedError;

    // Polynomial curve fitting
    // auto PGain = 9000000+pow( pow(6000000, (1/127)), velocity);
    // auto PGain = (ull)round (2000000+pow( pow(15000000, (1.0/127 .0)), velocity));
    // auto PGain = (ull)round (800000.0+((15000000.0-80000.0)/127.0)*velocity);   //First Experiment
    //auto PGain = (ull)round (726984.13+73015.87*velocity);   //Third Experiment
    //auto PGain = (ull)round (3400000.0/7.0 + 800000.0/7.0*velocity);   //4 Experiment
    //  auto PGain = (ull)round (34600000.0/63.0 + 3200000.0/63.0*velocity);   //5 Experiment
//     auto PGain = (ull)round (17900000.0/63.0 + 7300000.0/63.0*velocity);
    auto PGain = (ull)round (293073.0*exp(0.0336401*velocity));
    // auto PGain = (ull)round (15000000.0);
    //auto PGain = (ull)round (15000000);
    //auto PGain = (ull)round (799999.9+pow(1.13846,velocity));
    auto DGain = (ull)round(100000);
    std::stringstream msg;
    msg << "The stiffness set to " << PGain;   //  << ", Damping set to " << DGain/1000;
    LOG_INFO(msg.str());
    m_striker.setControllerGain(PGain, DGain);

    return kNoError;
}


Error_t MotorController::changePosition(float value) {
    if (!m_bIsEnabled)
        return kNotInitializedError;

    int acc = 100000;
    std::stringstream msg;
    msg << "move to position = " << value << ", acc = " << acc;
    LOG_TRACE(msg.str());
    m_striker.moveToPosition( value, acc, false);

    return kNoError;
}





//        LOG_INFO("motor_status");
//        LOG_INFO(m_MotorStatus);

//    clock_t time_ = clock();
//
//
//    const double NUM_SECONDS = value/10;
//
//        int count = 1;
//
//        double time_counter = 0;
//
//        clock_t this_time = clock();
//        clock_t last_time = this_time;
//
//        printf("Gran = %ld\n", NUM_SECONDS * CLOCKS_PER_SEC);
//
//        while(m_MotorStatus)
//        {
////            this_time = clock();
////
////            time_counter += (double)(this_time - last_time);
////
////            last_time = this_time;
////
////            if(time_counter > (double)(NUM_SECONDS * CLOCKS_PER_SEC))
////            {
////                time_counter -= (double)(NUM_SECONDS * CLOCKS_PER_SEC);
////                LOG_INFO(this_time - last_time);
////                count++;
//                LOG_INFO(value);
//                LOG_INFO("value");
//            }
//
//             printf("DebugTime = %f\n", time_counter);



//
//    /////////////  Fri Dec 3
//    typedef unsigned long long ull;
//
//    if (!m_bIsEnabled)
//        return kNotInitializedError;
//
//
//
//    if (value%2==1){
//        LOG_INFO("HERE");
//
//        int position = 0;
//
//        auto acc = (ull)round(100000);
//
//        std::stringstream msg;
//        msg << "move to position = " << position << ", acc = " << acc;
//        LOG_TRACE(msg.str());
//        position = motor_status * sin(w*t)
//        m_striker.moveToPosition( position, acc, false );
//    }
//    else{
//        int position = -700;
//
//        auto acc = (ull)round(100000);
//
//        std::stringstream msg;
//        msg << "move to position = " << position << ", acc = " << acc;
//        LOG_TRACE(msg.str());
//
//        m_striker.moveToPosition( position, acc, false);
//    }






//    auto position = (ull)round (velocity*5);
//
//    auto acc = (ull)round(100000);





//    std::stringstream msg;
//    msg << "move to position = " << position << ", acc = " << acc;
//    LOG_TRACE(msg.str());
//
//    m_striker.moveToPosition( position, acc, false);





