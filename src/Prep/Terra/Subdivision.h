#ifndef SUBDIVISION_INCLUDED // -*- C++ -*-
#define SUBDIVISION_INCLUDED

#include "Quadedge.h"

#include <iostream>
#include <vector>

namespace Terra {

class Subdivision;

class Triangle : public Labelled {
    Edge *anchor;
    Triangle *next_face;

public:

    Triangle(Edge *e, int t=0)
    {
	token = t;
	reshape(e);
    }
    
    virtual ~Triangle()
    {
        
    }

    Triangle *linkTo(Triangle *t) { next_face = t; return this; }
    Triangle *getLink() { return next_face; }
    Edge *getAnchor() { return anchor; }
    void dontAnchor(Edge *e);

    void reshape(Edge *e);

    virtual void update(Subdivision&); // called  to update stuff

    const Vec2& point1() const { return anchor->Org(); }
    const Vec2& point2() const { return anchor->Dest(); }
    const Vec2& point3() const { return anchor->Lprev()->Org(); }
};

typedef void (*edge_callback)(Edge *, void *);
typedef void (*face_callback)(Triangle&, void *);

typedef std::vector<Triangle*> TriangleVec;
typedef std::vector<Triangle*>::iterator TriangleVecIterator;

typedef std::vector<Edge*> EdgeVec;
typedef std::vector<Edge*>::iterator EdgeVecIterator;


class Subdivision {
private:
    Edge *startingEdge;
    Triangle *first_face;

    EdgeVec     edges;

protected:

    TriangleVec triangles;
    void initMesh(const Vec2&, const Vec2&, const Vec2&, const Vec2&);
    Subdivision();
    ~Subdivision();

    Edge *makeEdge();
    Edge *makeEdge(const Vec2& org, const Vec2& dest);

    virtual Triangle *allocFace(Edge *e);
    Triangle& makeFace(Edge *e);


    void deleteEdge(Edge *);
    Edge *connect(Edge *a, Edge *b);
    void swap(Edge *e);

    //
    // Some random functions
    boolean ccwBoundary(const Edge *e);
    boolean onEdge(const Vec2&, Edge *);

public:

    Subdivision(Vec2& a, Vec2& b, Vec2& c, Vec2& d) { initMesh(a,b,c,d); }

    //
    // virtual functions for customization
    virtual boolean shouldSwap(const Vec2&, Edge *);


    boolean isInterior(Edge *);

    Edge *spoke(Vec2&, Edge *e);
    void optimize(Vec2&, Edge *);

    Edge *locate(const Vec2& x)  { return locate(x, startingEdge); }
    Edge *locate(const Vec2&, Edge *hint);
    Edge *insert(Vec2&, Triangle *t=NULL);

    void overEdges(edge_callback, void *closure=NULL);
    void overFaces(face_callback, void *closure=NULL);
};


inline std::ostream& operator<<(std::ostream& out, Triangle& t)
{
    return out << "Triangle("<< t.point1() << " " << t.point2() << " "
	       << t.point3() << ")";
}

}; // namespace Terra

#endif
