#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

// For most CGAL functions, we will use exact constructions with square root
// square root is needed for CGAL::bisector()

typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt EPECKernel;
typedef EPECKernel::Point_2                                         EPECPoint_2;
typedef EPECKernel::Direction_2                                     EPECDirection_2;
typedef CGAL::Line_2<EPECKernel>                                    EPECLine_2;
typedef CGAL::Ray_2<EPECKernel>                                     EPECRay_2;
typedef CGAL::Vector_2<EPECKernel>                                  EPECVector_2;
typedef CGAL::Segment_2<EPECKernel>                                 EPECSegment_2;
