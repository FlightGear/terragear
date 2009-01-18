// contour.hxx - declaration of VpfContour

#ifndef __CONTOUR_HXX
#define __CONTOUR_HXX 1

#include "vpfbase.hxx"
#include "component.hxx"
#include <string>
#include <vector>

class VpfTable;
struct VpfPoint;

class VpfContour : public VpfComponent
{
public:
  VpfContour (const VpfContour &contour);
  virtual ~VpfContour ();

  virtual int getPointCount () const;
  virtual const VpfPoint getPoint (int index) const;

protected:

  struct line_info
  {
    bool isLR;
    int id;
    int offset;
    int size;
  };

  friend class VpfPolygon;

  VpfContour (int faceId, int startEdge, const std::string &path,
	      VpfTableManager &tableManager);

  virtual const VpfTable &getEDG () const;
  virtual const std::vector<line_info> &getLines() const;

private:

  int _polygon_id;
  int _start_edge;
  mutable int _nPoints;
  mutable const VpfTable * _edg;
  mutable std::vector<line_info> * _lines;
};

#endif

// end of contour.hxx
