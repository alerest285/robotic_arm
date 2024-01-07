#ifndef ROBOTIC_ARM_LOGGING_H
#define ROBOTIC_ARM_LOGGING_H

namespace robotic_arm {

enum class LoggingEnum {FATAL, ERROR, WARN, INFO, DEBUG};

typedef void (*LoggingCallback)(LoggingEnum, String);

String LoggingEnumToString(LoggingEnum level) {
    switch (level) {
        case LoggingEnum::DEBUG: return "DEBUG";
        case LoggingEnum::INFO:  return "INFO";
        case LoggingEnum::WARN:  return "WARN";
        case LoggingEnum::ERROR: return "ERROR";
        case LoggingEnum::FATAL: return "FATAL";
        default:    return "UNKNOWN";
    }
}

} // namespace robotic_arm

#endif // ROBOTIC_ARM_LOGGING_H
