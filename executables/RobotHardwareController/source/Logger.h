#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
/**
 * @todo write docs
 */

class Logger
{
    public:
        
        Logger();
        Logger(const std::string& path, unsigned logLevel = 0);
    
        inline std::ostream &log(unsigned logLevel){
            if (logLevel <= m_logLevel){
                return m_fileStream;                
            }else{
                return m_discardStream;                
            }
            
        }
        
    protected:
    
        unsigned m_logLevel;
        std::ofstream m_fileStream;
        std::ofstream m_discardStream;

};


extern Logger logger;

#endif // LOGGER_H
