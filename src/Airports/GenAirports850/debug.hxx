#include <map>
#include <string>
#include <cstring>
#include <vector>

#include <simgear/threads/SGThread.hxx>
#include <simgear/debug/logstream.hxx>

#ifndef __DEBUG_HXX__
#define __DEBUG_HXX__

/* a debug map maps ICAOs to a vector of indexs we want to debug */
typedef std::map<std::string, std::vector<int>, std::less<std::string> > debug_map;
typedef debug_map::iterator debug_map_iterator;
typedef debug_map::const_iterator debug_map_const_iterator;

/* We'll need a mutex to print log messages 1 at a time */
extern SGMutex logMutex;

/* This map maps thread IDs to ICAO prefixes */
extern std::map<long, std::string> thread_prefix_map;

extern void DebugRegisterPrefix( const std::string& prefix );
extern std::string DebugTimeToString(time_t& tt);

#define GENAPT_LOG(C,P,M)  do {                                                                                 \
    logstream& __tmplogstreamref(sglog());                                                                      \
    if(__tmplogstreamref.would_log(C,P)) {                                                                      \
        logMutex.lock();                                                                                        \
        __tmplogstreamref << loglevel(C,P) << thread_prefix_map[SGThread::current()] << ":" << M << std::endl;  \
        logMutex.unlock();                                                                                      \
        }                                                                                                       \
    } while(0)

#endif