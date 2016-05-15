#ifndef __TG_MUTEX_H__
#define __TG_MUTEX_H__

#include <simgear/threads/SGThread.hxx>

#define DEBUG_LOCKS (0)

class tgMutex : public SGMutex
{
public:
    tgMutex() : SGMutex()
    {
        held = 0;
    }

    void lock( void )
    {
#if DEBUG_LOCKS
        if ( held ) {
            SG_LOG( SG_GENERAL, SG_INFO, "Thread " << SGThread::current() << " requesting lock held by " << held << " wait " );
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "Thread " << SGThread::current() << " requesting unheld lock" );
        }
#endif

        SGMutex::lock();

#if DEBUG_LOCKS
        held = SGThread::current();
        SG_LOG( SG_GENERAL, SG_INFO, "Thread " << held << " has lock" );
#endif
    }

    void unlock( void )
    {
#if DEBUG_LOCKS
        if ( held ) {
            SG_LOG( SG_GENERAL, SG_INFO, "Thread " << SGThread::current() << " releasing lock held by " << held );
        } else {
            SG_LOG( SG_GENERAL, SG_INFO, "Thread " << SGThread::current() << " releasing unheld lock" );
        }
#endif

        SGMutex::unlock();

#if DEBUG_LOCKS
        held = 0;
#endif
    }

private:
    long held;
};

#endif /* __TG_MUTEX_H__ */