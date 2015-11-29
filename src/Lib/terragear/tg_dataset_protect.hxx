#include <map>

#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGGuard.hxx>
#include <simgear/debug/logstream.hxx>

// This file is used to serialize access to GDAL DataSets.
// datasets should only be open by one thread at a time
// we have a potentially HUGE number of datasets, however, as
// each tile is represented by one.

// we CAN open multiple datasets at the same time.

// so we keep a map of tiles. 
// if an entry is not in the map, it is available to use.
// if an entry IS in the map, we will queue up on a condition
// variable until it is ready

// when a tile becomes available, we will signal a waiting task
// so it can use it.

// when the last task is finished with a tile,  it is remved from 
// the map.


// structure containg information about a tile
// in use, number of waiters, etc...
struct tileInfo {
public:
    tileInfo( unsigned long id ) : tid( id ), numWaiting(1), inUse( true ) {}

    void AddWaiter( void ) {
	SGGuard<SGMutex> g(mutex);

        numWaiting++;
        while ( inUse ) {
            available.wait( mutex );
        }
    }

    bool RemoveWaiter( void ) {
	SGGuard<SGMutex> g(mutex);

        numWaiting--;
        inUse = false;
        available.signal();

	return ( numWaiting == 0 );
    }

    SGWaitCondition available;
    SGMutex         mutex;

    unsigned long   tid;
    int             numWaiting;     // when this is 0, we can remove tileInfo from the map
    bool            inUse;          // condition variable - cleared on release
};

typedef std::map< unsigned long, tileInfo* >	tile_map;

class tgDatasetAcess
{
public:
    tgDatasetAcess(void) {}

    // whenever you want to write to a tile, you need to Request it
    void Request( unsigned long tileId ) {
        mutex.lock();

        tile_map::iterator it = waitingTasks.find( tileId );
        if ( it == waitingTasks.end() ) {
            // tile is not in the map, therefore it is not in use.
            // create tileInfo, and insert into the map
            waitingTasks[tileId] = new tileInfo( tileId );
            // we can continue to use it - we're the first to ask for it
            // so no call to AddWaiter

            mutex.unlock();
        } else {
            // tile is in the map, so it is already in use
            // wait for it - this will block
            tileInfo* ti = it->second;

            // release our mutex before adding waiter
            mutex.unlock();
            ti->AddWaiter();

            // once AddWaiter returns, we have the tile for ourselves
        }
    }

    // whenever you finish writing to a tile, you need to Release it
    void Release( unsigned long tileId ) {
        SGGuard<SGMutex> g(mutex);

        tile_map::iterator it = waitingTasks.find( tileId );
        if ( it != waitingTasks.end() ) {
            // tile is already in use ( as we are using it... duh )
            tileInfo* ti = it->second;

            // RemoveWaiter returns true if there are no more waiters
            if ( ti->RemoveWaiter() ) {
                // if we were the only one using it, 
                // remove from the map, and delete
                delete ti;
                waitingTasks.erase( it );
            }
        } else {

            // uh-oh - this shouldn't happen
            SG_LOG( SG_GENERAL, SG_INFO, "tgDatasetAccess::Released tile " << tileId << " NOT IN MAP - ERROR " );
        }
    }

private:
    SGMutex  mutex;
    tile_map waitingTasks;
};
