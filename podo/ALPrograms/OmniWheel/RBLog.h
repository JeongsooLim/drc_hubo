#ifndef RBLOG_H
#define RBLOG_H

#include <sstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/time.h>

#include <QString>
#include <QTextStream>



inline std::string NowTime(){
    time_t tm_time;
    struct tm *st_time;
    char buff[30];

    time(&tm_time);
    st_time = localtime(&tm_time);
    //strftime(buff, 100, "%Y-%m-%d  %p%l:%M:%S", st_time);
    strftime(buff, 100, "OmniWheel\t%M:%S", st_time);

    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[50] = {0};
    sprintf(result, "%s.%03ld", buff, (long)tv.tv_usec/1000);
    //sprintf(result, "%s %03ld", buff, (long)tv.tv_usec/1000);
    return result;
}



enum TLogLevel {logERROR, logSUCCESS, logWARNING, logINFO, logDEBUG, logDEBUG1, logDEBUG2, logDEBUG3, logDEBUG4};

template <typename T>
class Log
{
public:
    Log();
    virtual ~Log();
    std::ostringstream& Get(TLogLevel level = logINFO);
public:
    static TLogLevel& ReportingLevel();
    static std::string ToString(TLogLevel level);
    static TLogLevel FromString(const std::string& level);
protected:
    std::ostringstream os;
private:
    Log(const Log&);
    Log& operator =(const Log&);
};

template <typename T>
Log<T>::Log()
{
}

template <typename T>
std::ostringstream& Log<T>::Get(TLogLevel level)
{
    if(level == logERROR){
        os << "\033[1;31m";
    }else if(level == logSUCCESS){
        os << "\033[1;32m";
    }else if(level == logWARNING){
        os << "\033[1;33m";
    }
    os << "- " << NowTime();
    os << " " << ToString(level) << ": ";
    os << std::string(level >= logDEBUG ? level - logDEBUG : 0, '\t');
    return os;
}

template <typename T>
Log<T>::~Log()
{
    os << "\033[0m" << std::endl;
    T::Output(os.str());
}

template <typename T>
TLogLevel& Log<T>::ReportingLevel()
{
    static TLogLevel reportingLevel = logDEBUG4;
    return reportingLevel;
}

template <typename T>
std::string Log<T>::ToString(TLogLevel level)
{
    static const char* const buffer[] = {"ERROR  ", "SUCCESS", "WARNING", "INFO   ", "DEBUG  ", "DEBUG1 ", "DEBUG2 ", "DEBUG3 ", "DEBUG4 "};
    return buffer[level];
}

template <typename T>
TLogLevel Log<T>::FromString(const std::string& level)
{
//    if (level == "DEBUG4 ")
//        return logDEBUG4;
//    if (level == "DEBUG3 ")
//        return logDEBUG3;
//    if (level == "DEBUG2 ")
//        return logDEBUG2;
//    if (level == "DEBUG1 ")
//        return logDEBUG1;
//    if (level == "DEBUG  ")
//        return logDEBUG;
//    if (level == "INFO   ")
//        return logINFO;
//    if (level == "WARNING")
//        return logWARNING;
//    if (level == "ERROR  ")
//        return logERROR;
//    Log<T>().Get(logWARNING) << "Unknown logging level '" << level << "'. Using INFO level as default.";
    return logINFO;
}

class Output2FILE
{
public:
    static FILE*& Stream();
    static void Output(const std::string& msg);
};

inline FILE*& Output2FILE::Stream()
{
    static FILE* pStream = stderr;
    return pStream;
}

inline void Output2FILE::Output(const std::string& msg)
{
    FILE* pStream = Stream();
    if (!pStream)
        return;
    fprintf(pStream, "%s", msg.c_str());
    fflush(pStream);
}


class FILELog : public Log<Output2FILE> {};

#ifndef FILELOG_MAX_LEVEL
#define FILELOG_MAX_LEVEL logDEBUG4
#endif

#define FILE_LOG(level) \
    if (level > FILELOG_MAX_LEVEL) ;\
    else if (level > FILELog::ReportingLevel() || !Output2FILE::Stream()) ; \
    else FILELog().Get(level)

#endif // RBLOG_H
