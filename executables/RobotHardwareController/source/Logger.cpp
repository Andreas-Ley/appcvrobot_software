#include "Logger.h"
#include "config.h"

Logger::Logger(): m_logLevel(logLevel){
    m_fileStream.open(logFile, std::fstream::out | std::fstream::app);    
}

Logger::Logger(const std::string& path, unsigned logLevel):
    m_logLevel(logLevel)
{
    m_fileStream.open(path.c_str(), std::fstream::out | std::fstream::app);
}

Logger logger;
