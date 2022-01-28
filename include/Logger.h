//
// Created by Raghavasimhan Sankaranarayanan on 6/20/20.
//

#ifndef GOOGLEDRUMMINGARM_LOGGER_H
#define GOOGLEDRUMMINGARM_LOGGER_H

// If debug output is defined, always use the trace logging level
#ifdef DEBUG_OUTPUT
#undef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include "Defines.h"

#include "pch.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#define LOG_TRACE(...)      SPDLOG_TRACE    (__VA_ARGS__)
#define LOG_DEBUG(...)      SPDLOG_DEBUG    (__VA_ARGS__)
#define LOG_INFO(...)       SPDLOG_INFO     (__VA_ARGS__)
#define LOG_WARN(...)       SPDLOG_WARN     (__VA_ARGS__)
#define LOG_ERROR(...)      SPDLOG_ERROR    (__VA_ARGS__)
#define LOG_CRITICAL(...)   SPDLOG_CRITICAL (__VA_ARGS__)

class Logger {
public:
    enum Level
    {
        trace = SPDLOG_LEVEL_TRACE,
        debug = SPDLOG_LEVEL_DEBUG,
        info = SPDLOG_LEVEL_INFO,
        warn = SPDLOG_LEVEL_WARN,
        err = SPDLOG_LEVEL_ERROR,
        critical = SPDLOG_LEVEL_CRITICAL,
        off = SPDLOG_LEVEL_OFF,

        kNumLogLevels
    };

    ~Logger() {
        spdlog::shutdown();
    }

    static void init() {
        spdlog::set_pattern("%^[%r]\t[%s]\t[line %#]\t[---%l---]\t%v%$");
#ifdef SPDLOG_ACTIVE_LEVEL
        spdlog::set_level((spdlog::level::level_enum)SPDLOG_ACTIVE_LEVEL);
#endif
        LOG_INFO("Logger initialized");
    }

//    template<typename T>
//    static void pprintArray(T* array, int size) {
//        for (int i=0; i<size; i++) {
//            std::cout << array[i] << " ";
//        }
//        std::cout << std::endl;
//    }
};

#endif //GOOGLEDRUMMINGARM_LOGGER_H
