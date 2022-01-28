//
// Created by Raghavasimhan Sankaranarayanan on 5/8/21.
//

#ifndef GUITARBOT_ERRORDEF_H
#define GUITARBOT_ERRORDEF_H

enum Error_t
{
    kNoError,

    kFileOpenError,
    kFileAccessError,
    kFileWriteError,
    kFileCloseError,
    kFileExtensionError,
    kFileReadError,
    kFileParseError,

    kFunctionInvalidArgsError,
    kFunctionExecOrderError,

    kNotInitializedError,
    kNotImplementedError,
    kFunctionIllegalCallError,
    kInvalidString,

    kGetValueError,
    kSetValueError,
    kOutOfRangeError,

    kOutOfBoundsError,

    kMemError,
    kNotEnoughMemoryError,
    kNotEnoughSamplesError,

    kBufferEmptyError,
    kBufferFullError,

    kUnknownError,

    kPandaError,

    kNullError,

    kNumErrors
};

enum PandaException_t {
    kNoException,
    kCommandException,
    kNetworkException,
    kControlException,
    kInvalidOperationException,
    kRealtimeException,
    kInvalidArgumentException,
    kModelException,
    kIncompatibleVersionException,
    kUnknownException,

    kNotInitializedException,

    kNumExceptions
};

#endif //GUITARBOT_ERRORDEF_H
