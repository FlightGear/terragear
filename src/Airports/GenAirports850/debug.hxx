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

/* This map maps thread IDs to ICAO prefixes */
extern std::map<long, std::string> thread_prefix_map;

extern void DebugRegisterPrefix( const std::string& prefix );
extern std::string DebugTimeToString(time_t& tt);

#define TG_LOG(C,P,M)  do {                                         \
    if(sglog().would_log(C,P, __FILE__, __LINE__, __FUNCTION__)) {                                    \
        std::ostringstream os;                                      \
        os << thread_prefix_map[SGThread::current()] << ":" << M;   \
        sglog().log(C, P, __FILE__, __LINE__, __FUNCTION__, os.str());            \
    }                                                               \
} while(0)
#endif
