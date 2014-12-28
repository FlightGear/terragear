#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

// For most CGAL functions, we will use exact constructions exact predicates
// square root is needed for CGAL::bisector(), but ECEP with SR has HUGE performance hit

typedef CGAL::Exact_predicates_exact_constructions_kernel           EPECKernel;
typedef EPECKernel::Point_2                                         EPECPoint_2;
typedef EPECKernel::Direction_2                                     EPECDirection_2;
typedef CGAL::Line_2<EPECKernel>                                    EPECLine_2;
typedef CGAL::Ray_2<EPECKernel>                                     EPECRay_2;
typedef CGAL::Vector_2<EPECKernel>                                  EPECVector_2;
typedef CGAL::Segment_2<EPECKernel>                                 EPECSegment_2;

typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt EPECSRKernel;
typedef EPECSRKernel::Point_2                                       EPECSRPoint_2;
typedef EPECSRKernel::Direction_2                                   EPECSRDirection_2;
typedef CGAL::Line_2<EPECSRKernel>                                  EPECSRLine_2;
typedef CGAL::Ray_2<EPECSRKernel>                                   EPECSRRay_2;
typedef CGAL::Vector_2<EPECSRKernel>                                EPECSRVector_2;
typedef CGAL::Segment_2<EPECSRKernel>                               EPECSRSegment_2;

typedef CGAL::Simple_cartesian<double>                              INEXACTKernel;

#include <CGAL/Cartesian_converter.h>
typedef CGAL::Cartesian_converter<EPECKernel,INEXACTKernel>         EPEC_to_double;
typedef CGAL::Cartesian_converter<EPECSRKernel,INEXACTKernel>       EPECSR_to_double;

