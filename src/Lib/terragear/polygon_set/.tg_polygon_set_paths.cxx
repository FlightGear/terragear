void tgPolygonSet::setVisited(cgalPoly_HeConstHandle he, std::vector<cgalPoly_HeConstHandle>& visited)
{
    visited.push_back( he );
}

bool tgPolygonSet::isVisited(cgalPoly_HeConstHandle he, std::vector<cgalPoly_HeConstHandle>& visited)
{
    // check the visited array
    bool found = false;
    
    for ( unsigned int i=0; i<visited.size(); i++ ) {
        if ( he == visited[i] || he->twin() == visited[i] ) {
            found = true;
            break;
        }
    }
    
    return found;
}
