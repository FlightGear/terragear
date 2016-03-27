// construct.hxx -- Class to manage the primary data used in the
//                  construction process
//
// Written by Curtis Olson, started May 1999.
//
// Copyright (C) 1999  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
//
// $Id: construct.hxx,v 1.13 2004-11-19 22:25:49 curt Exp $


#ifndef _TGCONSTRUCT_THIRD_HXX
#define _TGCONSTRUCT_THIRD_HXX

#ifndef __cplusplus                                                          
# error This library requires C++
#endif                                   

#include <simgear/threads/SGThread.hxx>
#include <simgear/threads/SGQueue.hxx>

#include <terragear/mesh/tg_mesh.hxx>

#include "priorities.hxx"

class tgConstructThird : public SGThread
{
public:
    // Constructor
    tgConstructThird( const std::string& priorities_file, SGLockedQueue<SGBucket>& q, SGMutex* l );

    // Destructor
    ~tgConstructThird();

    // paths
    void setPaths( const std::string& work, const std::string& dem, const std::string& share, const std::string& debug, const std::string& output );
    
private:
    virtual void run();

    // Ocean tile or not
    bool IsOceanTile()  { return isOcean; }

    // Load Data
    void loadElevation( const std::string& path );
    
    int loadMesh( const std::string& path );

private:
    TGAreaDefinitions           areaDefs;
    
    // construct stage to perform
    SGLockedQueue<SGBucket>&    workQueue;
    unsigned int                totalTiles;
    
    // paths
    std::string                 workBase;
    std::string                 demBase;
    std::string                 shareBase;
    std::string                 debugBase;
    std::string                 outputBase;

    // this bucket
    SGBucket                    bucket;

    tgMesh                      tileMesh;

    // ocean tile?
    bool                        isOcean;

    SGMutex*                    lock;
};

#endif // _TGCONSTRUCT_THIRD_HXX