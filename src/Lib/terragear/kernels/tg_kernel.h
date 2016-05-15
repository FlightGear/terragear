#ifndef __TG_KERNEL_HXX__
#define __TG_KERNEL_HXX__

#include <CGAL/Cartesian.h>

#include "tg_kernel_point_2.h"

template < typename K_, typename K_Base >
class TgCartesian_base : public K_Base::template Base<K_>::Type
{
  typedef typename K_Base::template Base<K_>::Type                      OldK;

public:
  typedef K_                                                            Kernel;
  typedef TgPointC2<Kernel>                                             Point_2;
  typedef TgConstruct_point_2<Kernel, OldK>                             Construct_point_2;
  typedef TgConstruct_coord_iterator<Kernel>                            Construct_cartesian_const_iterator_2;
  typedef TgConstruct_bbox_2<Kernel, typename OldK::Construct_bbox_2>   Construct_bbox_2;

  Construct_point_2 construct_point_2_object() const
  {
      return Construct_point_2(); 
  }

  Construct_bbox_2 construct_bbox_2_object() const
  { 
      return Construct_bbox_2(); 
  }

  Construct_cartesian_const_iterator_2 construct_cartesian_const_iterator_2_object() const
  { 
      return Construct_cartesian_const_iterator_2(); 
  }

  template < typename Kernel2 >
  struct Base {
      typedef TgCartesian_base<Kernel2, K_Base> Type; 
  };
};

template < typename FT_ >
struct TgKernel
  : public CGAL::Type_equality_wrapper<
                TgCartesian_base<TgKernel<FT_>, CGAL::Cartesian<FT_> >,
                TgKernel<FT_> >
{

};

#endif // __TG_KERNEL_HXX__