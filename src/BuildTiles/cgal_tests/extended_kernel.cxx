// extended_kernel.cxx -- CGAL extended kernel
//
// Written by Peter Sadrozinski, started May 2016.
//
// Copyright (C) 2016  Peter Sadrozinski
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

#include <CGAL/basic.h>
#include <CGAL/Filtered_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Origin.h>
#include <CGAL/Bbox_2.h>

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>

#include <CGAL/Snap_rounding_traits_2.h>
#include <CGAL/Snap_rounding_2.h>

template <typename K>
class MyPointC2 {
  typedef typename K::FT         FT;

private:
  FT  vec[2];
  int col;

public:
  MyPointC2()
    : col(0)
  {
    *vec = 0;
    *(vec+1) = 0;
  }
  MyPointC2(const FT x, const FT y, int c = 0)
    : col(c)
  {
    *vec = x;
    *(vec+1) = y;
  }
  const FT& x() const  { return *vec; }
  const FT& y() const { return *(vec+1); }
  FT & x() { return *vec; }
  FT& y() { return *(vec+1); }
  int color() const { return col; }
  int& color() { return col; }
  bool operator==(const MyPointC2 &p) const
  {
    return ( *vec == *(p.vec) )  && ( *(vec+1) == *(p.vec + 1) && ( col == p.col) );
  }
  bool operator!=(const MyPointC2 &p) const
  {
      return !(*this == p);
  }
};

template <typename K, class ConstructBbox_2>
class MyConstruct_bbox_2 : public ConstructBbox_2 {
  
public:
  using ConstructBbox_2::operator();
  CGAL::Bbox_2 operator()(const MyPointC2<K>& p) const {
    return CGAL::Bbox_2(p.x(), p.y(), p.x(), p.y());
  }
};

template <typename K>
class MyConstruct_coord_iterator {
  typedef typename K::FT         FT;
public:
  typedef const FT*              result_type;

  const FT* operator()(const MyPointC2<K>& p) const
  {
    return &p.x();
  }
  const FT* operator()(const MyPointC2<K>& p, int)
  {
    const FT* pyptr = &p.y();
    pyptr++;
    return pyptr;
  }
};

template <typename K, typename OldK>
class MyConstruct_point_2
{
  typedef typename K::RT         RT;
  typedef typename K::Point_2    Point_2;
  typedef typename K::Line_2     Line_2;
  typedef typename Point_2::Rep  Rep;
public:
  typedef Point_2                result_type;
  // Note : the CGAL::Return_base_tag is really internal CGAL stuff.
  // Unfortunately it is needed for optimizing away copy-constructions,
  // due to current lack of delegating constructors in the C++ standard.
  Rep // Point_2
  operator()(CGAL::Return_base_tag, CGAL::Origin o) const
  { return Rep(o); }
  Rep // Point_2
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y) const
  { return Rep(x, y); }
  Rep // Point_2
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y, const RT& w) const
  { return Rep(x, y, w); }
  Point_2
  operator()(const CGAL::Origin&) const
  { return MyPointC2<K>(0, 0, 0); }
  Point_2
  operator()(const RT& x, const RT& y) const
  {
    return MyPointC2<K>(x, y, 0);
  }
  Point_2
  operator()(const Line_2& l) const
  {
    typename OldK::Construct_point_2 base_operator;
    Point_2 p = base_operator(l);
    return p;
  }
  Point_2
  operator()(const Line_2& l, int i) const
  {
    typename OldK::Construct_point_2 base_operator;
    return base_operator(l, i);
  }
  // We need this one, as such a functor is in the Filtered_kernel
  Point_2
  operator()(const RT& x, const RT& y, const RT& w) const
  {
    if(w != 1){
      return MyPointC2<K>(x/w, y/w, 0);
    } else {
      return MyPointC2<K>(x,y, 0);
    }
  }
};


// K_ is the new kernel, and K_Base is the old kernel
template < typename K_, typename K_Base >
class MyCartesian_base
  : public K_Base::template Base<K_>::Type
{
  typedef typename K_Base::template Base<K_>::Type                      OldK;

public:
  typedef K_                                                            Kernel;
  typedef MyPointC2<Kernel>                                             Point_2;
  //typedef MySegmentC2<Kernel>                                         Segment_2;
  typedef MyConstruct_point_2<Kernel, OldK>                             Construct_point_2;
  typedef const double*                                                 Cartesian_const_iterator_2;
  typedef MyConstruct_coord_iterator<Kernel>                            Construct_cartesian_const_iterator_2;
  typedef MyConstruct_bbox_2<Kernel, typename OldK::Construct_bbox_2>   Construct_bbox_2;

  Construct_point_2     construct_point_2_object() const
  {
      return Construct_point_2(); 
  }

  Construct_bbox_2      construct_bbox_2_object() const
  { 
      return Construct_bbox_2(); 
  }

  Construct_cartesian_const_iterator_2 construct_cartesian_const_iterator_2_object() const
  { 
      return Construct_cartesian_const_iterator_2(); 
  }

  template < typename Kernel2 >
  struct Base { 
      typedef MyCartesian_base<Kernel2, K_Base>  Type; 
  };
};

template < typename FT_ >
struct MyKernel
  : public CGAL::Type_equality_wrapper<
                MyCartesian_base<MyKernel<FT_>, CGAL::Cartesian<FT_> >,
                MyKernel<FT_> >
{

};

typedef MyKernel<double>                   MK;
typedef CGAL::Filtered_kernel_adaptor<MK>  K;

typedef CGAL::Arr_segment_traits_2<K>      arrTraits;
typedef arrTraits::Point_2                 arrPoint;
typedef arrTraits::Curve_2                 arrSegment;

typedef CGAL::Snap_rounding_traits_2<K>    srTraits;

typedef std::list<arrSegment>              srSegmentList;
typedef std::list<arrPoint>                srPolyline;
typedef std::list<srPolyline>              srPolylineList;

int main()
{
    srSegmentList  srInputSegs;
    srPolylineList srOutputSegs;

    arrPoint src(1.0, 1.0);
    arrPoint trg(2.0, 1.0);
    srInputSegs.push_back( arrSegment(src, trg) );

    CGAL::snap_rounding_2<srTraits, srSegmentList::const_iterator, srPolylineList>
    (srInputSegs.begin(), srInputSegs.end(), srOutputSegs, 0.0000002, true, false, 5);
}