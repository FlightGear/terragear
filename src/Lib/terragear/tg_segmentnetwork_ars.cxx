#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <CGAL/assertions.h>
#include <CGAL/squared_distance_2.h>

#include <simgear/math/SGMath.hxx>
#include <simgear/debug/logstream.hxx>

#include "tg_segmentnetwork.hxx"
#include "tg_shapefile.hxx"

bool tgSegmentNetwork::ArbitraryRayShoot( const segnetVertexHandle from, double course, double dist, segnetPoint& minPoint, unsigned int finger_id, const char* dirname) const
{
    segnetPoint originPoint = from->point();
    SGGeod geodCurr = SGGeod::fromDeg( CGAL::to_double(originPoint.x()), CGAL::to_double(originPoint.y()) );
    SGGeod geodNext = SGGeodesy::direct( geodCurr, course, 5.0 );
    bool   found    = false;
    
    Halfedge_set crossed_edges;
    Result_type  out_obj;

#if DEBUG_FINGER_EXTENSION            
    char   layer[128];
    
    tgSegment segray( geodCurr, geodNext );
    sprintf(layer, "finger_extension_front_%04d", finger_id);
    tgShapefile::FromSegment( segray, false, datasource, layer, "poly" );            
#endif
    
    segnetPoint next( geodNext.getLongitudeDeg(), geodNext.getLatitudeDeg() );            
    segnetLine  rayLine( originPoint, next );
    
    out_obj = _walk_from_vertex(from, next, false, crossed_edges);
        
    minPoint = next;
    for ( Halfedge_set::iterator it = crossed_edges.begin(); it != crossed_edges.end(); it++ ) {
        segnetHalfedgeHandle he = *it;
                
#if DEBUG_FINGER_EXTENSION
        SGGeod geodSource = SGGeod::fromDeg( CGAL::to_double( he->source()->point().x()), 
                                             CGAL::to_double( he->source()->point().y()) );
        SGGeod geodTarget = SGGeod::fromDeg( CGAL::to_double( he->target()->point().x()), 
                                             CGAL::to_double( he->target()->point().y()) );
        tgSegment seg( geodSource, geodTarget );
                
        sprintf(layer, "finger_extension_crossed_%04d", finger_id);
        tgShapefile::FromSegment( seg, false, datasource, layer, "poly" );
#endif            

        // find the intersection and distance
        segnetSegment heSeg( he->source()->point(), he->target()->point() );
                
        CGAL::Object result = CGAL::intersection( heSeg, rayLine );
        if ( const segnetPoint *iPoint = CGAL::object_cast<segnetPoint>(&result)) {
            // compare to remember minimal
            CGAL::Comparison_result cr =  CGAL::compare_distance_to_point( originPoint, *iPoint, minPoint );
            if ( cr == CGAL::SMALLER ) {
                minPoint = *iPoint;
                found = true;
            }
        }
    }
    
    // TODO
    // verify projection distance is close
    // we may need to iterate back to find the edge we can project to                
        
    return found;
}

Result_type tgSegmentNetwork::_find_face_around_vertex(segnetVertexHandle vh, const segnetPoint& p, bool& new_vertex) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());

    new_vertex = false;
    
    // Create an x-monotone curve connecting the point associated with the
    // vertex vp and the query point p.
    const segnetPoint&      vp = vh->point();
    segnetXMonotoneCurve    seg = m_traits->construct_x_monotone_curve_2_object()(vp, p);
    const bool              seg_dir_right = (m_traits->compare_xy_2_object()(vp, p) == CGAL::SMALLER);
    
    // Get the first incident halfedge around v and the next halfedge.
    Halfedge_around_vertex_const_circulator  first = vh->incident_halfedges();
    Halfedge_around_vertex_const_circulator  curr, next;
    bool                                     equal_curr = false;
    
    next = curr = first;
    ++next;
    
    if (next == curr) {
        // The vertex has a single incident edge. Check if its associated
        // curve equals seg next to vp.
        if (seg_dir_right && curr->direction() == CGAL::ARR_RIGHT_TO_LEFT) {
            // Both curves are defined to the right of vp:
            equal_curr = (m_traits->compare_y_at_x_right_2_object()(curr->curve(), seg, vp) == CGAL::EQUAL);
        } else if (!seg_dir_right && curr->direction() == CGAL::ARR_LEFT_TO_RIGHT) {
            // Both curves are defined to the left of vp:
            equal_curr = (m_traits->compare_y_at_x_left_2_object()(curr->curve(), seg, vp) == CGAL::EQUAL);
        }
        
        // In case the curves are not equal, just return the incident face of
        // the single halfegde (note that this is also the incident face of its
        // twin, as v is the tip of an "antenna").
        if (!equal_curr) {
            CGAL_assertion(curr->face() == curr->twin()->face());
            return Result::make_result(curr->face());
        }
    } else {
        // Traverse the halfedges around v until we find the pair of adjacent
        // halfedges such as seg is located clockwise in between them.
        segnetTraitsAdaptor::Is_between_cw_2  is_between_cw = m_traits->is_between_cw_2_object();
        bool                                  eq_curr, eq_next;
        
        while (!is_between_cw(seg, seg_dir_right, curr->curve(), 
                              (curr->direction() == CGAL::ARR_RIGHT_TO_LEFT),
                              next->curve(),
                              (next->direction() == CGAL::ARR_RIGHT_TO_LEFT),
                              vp, eq_curr, eq_next))
        {
            // Break the loop if seg equals one of the halfegdes next to v.
            if (eq_curr) {
                equal_curr = true;
                break;
            }
            
            if (eq_next) {
                curr = next;
                equal_curr = true;
                break;
            }
            
            // Move to the next pair of incident halfedges.
            curr = next;
            ++next;
            
            // Guard for an infinitive loop, in case we have completed a full
            // traversal around v without locating a place for seg.
            if (curr == first) {
                CGAL_error_msg("Completed a full cycle around v without locating seg.");
                return Result::default_result();
            }
        }
        
        // In case seg is not equal to curr's curve, just return the incident face
        // of the halfegde we have located.
        if (!equal_curr)
            return Result::make_result(curr->face());
    }
    
    // If we reached here, seg overlaps the curve associated with curr next to
    // the vertex v. We first check if p equals the other end-vertex of this
    // halfedge.
    if (m_traits->equal_2_object()(p, curr->source()->point())) {
        // In this case p equals the source point of the edge.
        return Result::make_result(curr->source());
    }
    
    // Check whether p lies on the curve associated with the edge.
    if (m_traits->is_in_x_range_2_object()(curr->curve(), p) && 
        m_traits->compare_y_at_x_2_object()(p, curr->curve()) == CGAL::EQUAL)
    {
        // p is located on the interior of the edge.
        segnetHalfedgeHandle he = curr;
        return Result::make_result(he);
    }
    
    // In this case, the source vertex of the current edge is closer
    // to the query point p.
    new_vertex = true;
    return Result::make_result(curr->source());
}

segnetHalfedgeHandle tgSegmentNetwork::_intersection_with_ccb(Ccb_halfedge_const_circulator circ, const segnetXMonotoneCurve& seg,
                                                              const segnetPoint& p, bool p_is_left, Halfedge_set& crossed_edges,
                                                              bool& is_on_edge, bool& is_target, bool& cv_is_contained_in_seg,
                                                              segnetVertexHandle& new_vertex) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());

    is_on_edge = false;
    is_target  = false;
    
    // Go over the CCB.
    segnetTraitsAdaptor::Is_in_x_range_2 is_in_x_range = m_traits->is_in_x_range_2_object();
    Ccb_halfedge_const_circulator        curr = circ , temp_circ;
    const segnetHalfedgeHandle           invalid_he;
    segnetHalfedgeHandle                 he;
    bool                                 cv_and_seg_overlap;
    do {
        he = curr;
        // Skip fictitious halfedges.
        if (he->is_fictitious()) {
            ++curr;
            continue;
        }
        
        // Check if we have already crossed the current halfedge (or its twin).
        // If so, we do not cross it again.
        if (crossed_edges.count(he) != 0) {
            ++curr;
            continue;
        }
        
        if (m_traits->is_vertical_2_object()(he->curve())) {
            if (is_in_x_range(he->curve(), p)) {
                if (m_traits->compare_y_at_x_2_object()(p, he->curve()) == CGAL::EQUAL) {
                    // special treatment in case the query point is on a vertical curve
                    is_on_edge = true;
                    return _in_case_p_is_on_edge(he, crossed_edges, p, is_target);
                }
            }
        }
        
        // Check if the x-range of the curve associated with the current edge
        // does not overlap the x-range of seg, the two curves cannot intersect.
        if (! is_in_x_range(he->curve(), seg)) {
            ++curr;
            continue;
        }
        cv_and_seg_overlap = false;
        cv_is_contained_in_seg = false;
        
        // Check whether the current curve intersects seg an odd number of times.
        if (_have_odd_intersections(he->curve(), seg, p_is_left, is_on_edge, cv_and_seg_overlap, cv_is_contained_in_seg) && 
                                    !(cv_and_seg_overlap || cv_is_contained_in_seg) ) {
            // Check if the query point lies on the current edge, or whether
            // it lies in its interior.
            if (is_on_edge) {
                return _in_case_p_is_on_edge(he,crossed_edges,p,is_target);
            }
            
            if ((!curr->target()->is_at_open_boundary()) && 
                is_in_x_range(seg, curr->target()->point())) {
                // if the target point of curr is located on seg
                // we should walk from it to the query point
                if (m_traits->compare_y_at_x_2_object()(curr->target()->point(), seg) == CGAL::EQUAL)
                {
                    new_vertex = curr->target();
                }
            }
            else if ((!curr->source()->is_at_open_boundary()) && is_in_x_range(seg , curr->source()->point() )) {
                // if the source point of curr is located on seg
                // we should walk from it to the query point
                if (m_traits->compare_y_at_x_2_object()(curr->source()->point() , seg) == CGAL::EQUAL)
                {
                    new_vertex = curr->source();
                }
            }
            
            // Return the halfedge we found, and mark that we have already crossed
            // it (as well as its twin).
            // the assumption is that each edge is crossed at most twice
            CGAL_assertion_msg(crossed_edges.count(he) < 2, "crossed_edges should contain each halfedge at most twice.");
            crossed_edges.insert(he);
            crossed_edges.insert(he->twin());
            return he;
        } else if (cv_and_seg_overlap || cv_is_contained_in_seg) {
            // Check if the query point lies on the current edge, or whether
            // it lies in its interior.
            if (cv_is_contained_in_seg) {
                // cv is contained in seg, obviously we crossed it
                // the assumption is that each edge is crossed at most twice
                CGAL_assertion_msg (crossed_edges.count(he) < 2, "crossed_edges should contain each halfedge at most twice.");
                crossed_edges.insert(he);
                crossed_edges.insert(he->twin());
                return invalid_he;
            }
            
            if (is_on_edge) {
                return _in_case_p_is_on_edge(he,crossed_edges,p,is_target);
            }
        }
        // Proceed to the next halfedge along the CCB.
        ++curr;
    } while (curr != circ);
    
    // If we reached here, we did not find any edge intersecting seg.
    return (invalid_he);
}

bool tgSegmentNetwork::_have_odd_intersections(const segnetXMonotoneCurve& cv, const segnetXMonotoneCurve& seg,
                                               bool p_is_left, bool& p_on_curve, bool& cv_and_seg_overlap, bool& cv_is_contained_in_seg) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());
    
    segnetTraitsAdaptor::Is_in_x_range_2 is_in_x_range = m_traits->is_in_x_range_2_object();
    p_on_curve = false;
    cv_and_seg_overlap = false;
    cv_is_contained_in_seg = false;
    
    // Use the left and right endpoints of the segment.
    const segnetPoint& seg_left = m_traits->construct_min_vertex_2_object()(seg);
    const segnetPoint& seg_right = m_traits->construct_max_vertex_2_object()(seg);
    
    // Use the left and right endpoints of the segment.
    segnetPoint cv_left;
    segnetPoint cv_right;
    bool cv_left_is_closed = m_traits->is_closed_2_object()(cv, CGAL::ARR_MIN_END);
    bool cv_right_is_closed = m_traits->is_closed_2_object()(cv, CGAL::ARR_MAX_END);
    if (cv_left_is_closed) {
        cv_left = m_traits->construct_min_vertex_2_object()(cv);
    }
    if (cv_right_is_closed) {
        cv_right = m_traits->construct_max_vertex_2_object()(cv);
    }
    if (cv_left_is_closed && cv_right_is_closed) {
        if (is_in_x_range(seg,cv_left) && is_in_x_range(seg,cv_right)) {
            if ((m_traits->compare_y_at_x_2_object()(cv_left, seg) == CGAL::EQUAL) &&
                (m_traits->compare_y_at_x_2_object()(cv_right, seg) == CGAL::EQUAL)) {
                // cv is contained in seg non of the answer true or false is correct
                // we must set a special flag to distinguish this case
                cv_is_contained_in_seg = true;
                return true;
            }
        }
    }
    
    // Check if the overlapping x-range of the two curves is trivial.
    // In this case, they cannot cross.
    if (cv_left_is_closed) {
        // Check if the left endpoint of cv has the same x-coordinate as the
        // right endpoint of seg.
        if (m_traits->compare_x_2_object()(m_traits->construct_min_vertex_2_object()(cv), seg_right) == CGAL::EQUAL) {
            if (!p_is_left && m_traits->compare_xy_2_object()(m_traits->construct_min_vertex_2_object()(cv), seg_right) == CGAL::EQUAL) {
                p_on_curve = true;
                return true;
            } else if (m_traits->is_vertical_2_object()(seg)) {
                // Special treatment for vertical segments.
                CGAL::Comparison_result res_l = m_traits->compare_y_at_x_2_object()(seg_left, cv);
                CGAL::Comparison_result res_r = m_traits->compare_y_at_x_2_object()(seg_right, cv);
                if ((p_is_left && res_l == CGAL::EQUAL) || (!p_is_left && res_r == CGAL::EQUAL)) {
                    p_on_curve = true;
                    return true;
                }
                return (res_l != res_r);
            }
            return false;
        }
    }
    if (cv_right_is_closed) {
        // Check if the right endpoint of cv has the same x-coordinate as the
        // left endpoint of seg.
        if (m_traits->compare_x_2_object()(m_traits->construct_max_vertex_2_object()(cv), seg_left) == CGAL::EQUAL) {
            if (p_is_left && m_traits->compare_xy_2_object()(m_traits->construct_max_vertex_2_object()(cv), seg_left) == CGAL::EQUAL) {
                p_on_curve = true;
                return true;
            } else if (m_traits->is_vertical_2_object()(seg)) {
                // Special treatment for vertical segments.
                CGAL::Comparison_result res_l = m_traits->compare_y_at_x_2_object()(seg_left, cv);
                CGAL::Comparison_result res_r = m_traits->compare_y_at_x_2_object()(seg_right, cv);
                
                if ((p_is_left && res_l == CGAL::EQUAL) || (! p_is_left && res_r == CGAL::EQUAL)) {
                    p_on_curve = true;
                    return true;
                }
                return (res_l != res_r);
            }
            return false;
        }
    }
    // Compare the two left ends of cv and seg.
    CGAL::Comparison_result    left_res;
    const CGAL::Arr_parameter_space  bx_l = m_traits->parameter_space_in_x_2_object()(cv, CGAL::ARR_MIN_END);
    if (bx_l == CGAL::ARR_LEFT_BOUNDARY) {
        // The left end of cv lies to the left of seg_left:
        // Compare this point to cv.
        left_res = m_traits->compare_y_at_x_2_object()(seg_left, cv);
    } else if (bx_l == CGAL::ARR_RIGHT_BOUNDARY) {
        // The left end of cv lies to the right of seg_left.
        // Compare the left endpoint of cv to seg.
        left_res = m_traits->compare_y_at_x_2_object()(m_traits->construct_min_vertex_2_object()(cv), seg);
        left_res = CGAL::opposite(left_res);
    } else {
        const CGAL::Arr_parameter_space  by_l = m_traits->parameter_space_in_y_2_object()(cv, CGAL::ARR_MIN_END);
        if (by_l == CGAL::ARR_BOTTOM_BOUNDARY) {
            // The left end of cv is at y = -oo, so cv obviously lies above it.
            left_res = CGAL::LARGER;
        } else if (by_l == CGAL::ARR_TOP_BOUNDARY) {
            // The left end of cv is at y = +oo, so cv obviously lies below it.
            left_res = CGAL::SMALLER;
        } else {
            // In this case cv has a valid left endpoint: Find the rightmost of
            // these two points and compare it to the other curve.
            CGAL::Comparison_result res = m_traits->compare_xy_2_object()(cv_left, seg_left);
            if (res != CGAL::LARGER) {
                left_res = m_traits->compare_y_at_x_2_object()(seg_left, cv);
                if (p_is_left && left_res == CGAL::EQUAL) {
                    // In this case the query point p, which is the left endpoint of seg,
                    // lies on cv.
                    p_on_curve = true;
                    return true;
                }
            } else {
                left_res = m_traits->compare_y_at_x_2_object()(cv_left, seg);
                left_res = CGAL::opposite(left_res);
            }
        }
    }
    if (left_res == CGAL::EQUAL) {
        // Compare the two curves to the right of their common left endpoint.
        if (is_in_x_range(cv,seg_left)) {
            left_res = m_traits->compare_y_at_x_right_2_object()(seg, cv, seg_left);
        } else if (is_in_x_range(seg,cv_left)) {
            left_res = m_traits->compare_y_at_x_right_2_object()(seg, cv, cv_left);
        } else {
            CGAL_error();
        }
        
        if (left_res == CGAL::EQUAL) {
            // RWRW: In this case we have an overlap ...
            // cv and seg overlap non of the answer true or false is correct
            // we must set a special flag to distinguish this case
            if (is_in_x_range(cv,( p_is_left ? seg_left : seg_right))) {
                if (m_traits->compare_y_at_x_2_object()((p_is_left ? seg_left : seg_right), cv) == CGAL::EQUAL) {
                    p_on_curve = true;
                }
            }
            // TODO - cerify - CGAL indenting was confusing here- possible bug?
            cv_and_seg_overlap = true;
            return true;
        }
    }
    // Compare the two right ends of cv and seg.
    CGAL::Comparison_result    right_res;
    const CGAL::Arr_parameter_space  bx_r = m_traits->parameter_space_in_x_2_object()(cv, CGAL::ARR_MAX_END);
    if (bx_r == CGAL::ARR_RIGHT_BOUNDARY) {
        // The right end of cv lies to the right of seg_right:
        // Compare this point to cv.
        right_res = m_traits->compare_y_at_x_2_object()(seg_right, cv);
    } else if (bx_r == CGAL::ARR_LEFT_BOUNDARY) {
        // The right end of cv lies to the left of seg_right.
        // Compare the right endpoint of cv to seg.
        right_res = m_traits->compare_y_at_x_2_object()(m_traits->construct_max_vertex_2_object()(cv), seg);
        right_res = CGAL::opposite(right_res);
    } else {
        const CGAL::Arr_parameter_space  by_r = m_traits->parameter_space_in_y_2_object()(cv, CGAL::ARR_MAX_END);
        if (by_r == CGAL::ARR_BOTTOM_BOUNDARY) {
            // The right end of cv is at y = -oo, so cv obviously lies above it.
            right_res = CGAL::LARGER;
        } else if (by_r == CGAL::ARR_TOP_BOUNDARY) {
            // The right end of cv is at y = +oo, so cv obviously lies below it.
            right_res = CGAL::SMALLER;
        } else {
            // In this case cv has a valid right endpoint: Find the leftmost of
            // these two points and compare it to the other curve.
            CGAL::Comparison_result res = m_traits->compare_xy_2_object()(cv_right, seg_right);
            
            if (res != CGAL::SMALLER) {
                right_res = m_traits->compare_y_at_x_2_object()(seg_right, cv);
                if (! p_is_left && right_res == CGAL::EQUAL) {
                    // In this case the query point p, which is the right endpoint of seg,
                    // lies on cv.
                    p_on_curve = true;
                    return true;
                }
            } else {
                right_res = m_traits->compare_y_at_x_2_object()(cv_right, seg);
                right_res = CGAL::opposite(right_res);
            }
        }
    }
    if (right_res == CGAL::EQUAL) {
        // Compare the two curves to the left of their common right endpoint.
        if (is_in_x_range(cv,seg_right)) {
            right_res = m_traits->compare_y_at_x_left_2_object()(seg, cv, seg_right);
        } else if (is_in_x_range(seg,cv_right)) {
            right_res = m_traits->compare_y_at_x_left_2_object()(seg, cv, cv_right);
        } else {
            CGAL_error();
        }
        
        if (right_res == CGAL::EQUAL) {
            // RWRW: In this case we have an overlap ...
            // cv and seg overlap non of the answer true or false is correct
            // we must set a special flag to distinguish this case
            if (is_in_x_range(cv, (p_is_left ? seg_left : seg_right))) {
                if (m_traits->compare_y_at_x_2_object()((p_is_left ? seg_left : seg_right), cv) == CGAL::EQUAL) {
                    p_on_curve = true;
                }
            }
            // TODO: see above
            cv_and_seg_overlap = true;
            return true;
        }
    }
    // The two curves intersect an odd number of times if the comparison
    // results at the two ends are not the same (this indicates that they
    // switch positions).
    return (left_res != right_res);
}
                        
segnetHalfedgeHandle tgSegmentNetwork::_in_case_p_is_on_edge(segnetHalfedgeHandle he, Halfedge_set& crossed_edges, const segnetPoint& p, bool& is_target) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());

    // cv and seg overlap, obviously we crossed it
    // the assumption is that each edge is crossed at most twice
    CGAL_assertion_msg(crossed_edges.count(he) < 2, "crossed_edges should contain each halfedge at most twice.");
    
    crossed_edges.insert(he);
    crossed_edges.insert(he->twin());
    
    // Check if p equals one of the edge end-vertices.
    if (!he->target()->is_at_open_boundary() && m_traits->compare_xy_2_object()(he->target()->point(), p) == CGAL::EQUAL) {
        // p is the target of the current halfedge.
        is_target = true;
    } else if (! he->source()->is_at_open_boundary() && m_traits->compare_xy_2_object()(he->source()->point(), p) == CGAL::EQUAL) {
        // Take the twin halfedge, so p equals its target.
        he = he->twin();
        is_target = true;
    }

    // Return the halfedge containing p.
    return he;
}
     
Result_type tgSegmentNetwork::_deal_with_curve_contained_in_segment(segnetHalfedgeHandle he, bool p_is_left, const segnetPoint& p, Halfedge_set& crossed_edges) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());

    // in this case we want to walk from to the query point from the nearest
    // vertex either the halfedge's source or target
    segnetVertexHandle  vh;
    bool target_is_left;
    
    if (m_traits->compare_xy_2_object()(he->source()->point(),he->target()->point()) == CGAL::LARGER) {
        target_is_left = true;
    } else {
        target_is_left = false;
    }
    
    if (p_is_left) {
        if (target_is_left) {
            vh = he->target();
        } else {
            vh = he->source();
        }
    } else {
        if (target_is_left) {
            vh = he->source();
        } else {
            vh = he->target();
        }
    }
    
    // vh is the closest vertex among the halfedge's end points
    return (_walk_from_vertex(vh, p, true, crossed_edges));
}
                                           
Result_type tgSegmentNetwork::_walk_from_vertex(segnetVertexHandle nearest_vertex, const segnetPoint& p, bool add_ce, Halfedge_set& crossed_edges) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());
    
    segnetVertexHandle vh = nearest_vertex;
    CGAL_assertion_msg(!vh->is_at_open_boundary(), "_walk_from_vertex() from a vertex at infinity.");
    
    // Check if the qurey point p conincides with the vertex.
    // if (m_traits->equal_2_object()(vh->point(), p))
    if (arr.geometry_traits()->equal_2_object()(vh->point(), p))
        return Result::make_result(vh);
    
    // In case of an isolated vertex, walk to from the face that contains
    // it toward the query point.
    if (vh->is_isolated()) {
        segnetFaceHandle  fh = vh->face();
        return (_walk_from_face(fh, vh->point(), p, crossed_edges));
    }
        
    if ( add_ce ) {
        // if we walk from a vertex this means we are crossing the
        // halfedges that form a corridor in which seg is going through
        Halfedge_around_vertex_const_circulator     first = vh->incident_halfedges();
        // Create an x-monotone curve connecting the point associated with the
        // vertex vp and the query point p.
        const segnetPoint&                          vp = vh->point();
        segnetXMonotoneCurve                        seg = m_traits->construct_x_monotone_curve_2_object()(vp, p);
        const bool                                  seg_dir_right = (m_traits->compare_xy_2_object()(vp, p) == CGAL::SMALLER);
        Halfedge_around_vertex_const_circulator     curr_iter = first;
        Halfedge_around_vertex_const_circulator     next_iter = curr_iter; ++next_iter;
        segnetTraitsAdaptor::Is_between_cw_2        is_between_cw = m_traits->is_between_cw_2_object();
        
        // Traverse the halfedges around vp until we find the pair of adjacent
        // halfedges such as seg is located clockwise in between them.
        do {
            bool eq_curr_iter, eq_next_iter;
            if (is_between_cw(seg, seg_dir_right, curr_iter->curve(),
                            (curr_iter->direction() == CGAL::ARR_RIGHT_TO_LEFT),
                            next_iter->curve(),
                            (next_iter->direction() == CGAL::ARR_RIGHT_TO_LEFT),
                            vp, eq_curr_iter, eq_next_iter))
            {
                // the assumption is that each edge is crossed at most twice
                CGAL_assertion_msg(crossed_edges.count (curr_iter) < 2,
                                    "crossed_edges should contain each halfedge at most twice.");
                CGAL_assertion_msg(crossed_edges.count (next_iter) < 2,
                                    "crossed_edges should contain each halfedge at most twice.");
                crossed_edges.insert(curr_iter);
                crossed_edges.insert(curr_iter->twin());
                crossed_edges.insert(next_iter);
                crossed_edges.insert(next_iter->twin());
                break;
            }
            ++curr_iter;
            ++next_iter;
        } while (curr_iter != first);
    }
    
    // Locate the face around the vertex that contains the curve connecting
    // the vertex and the query point.
    while (true) {
        bool new_vertex;
        Result_type obj = _find_face_around_vertex(vh, p, new_vertex);
        
        if (new_vertex) {
            // We found a vertex closer to p; Continue using this vertex.
            const segnetVertexHandle* p_vh = Result().template assign<segnetVertexHandle>(obj);
            CGAL_assertion(p_vh != NULL);
            vh = *p_vh;
            continue;
        }
        
        // If p is located on an edge or on a vertex, return the object
        // that wraps this arrangement feature.
        if (Result().template assign<segnetHalfedgeHandle>(obj) ||
            Result().template assign<segnetVertexHandle>(obj)) {
            return obj;
        }
        
        const segnetFaceHandle* p_fh = Result().template assign<segnetFaceHandle>(obj);
        if (p_fh) {
            // Walk to p from the face we have located:
            return _walk_from_face(*p_fh, vh->point(), p, crossed_edges);
        }
        
        CGAL_error_msg("_find_face_around_vertex() returned an unknown object.");
    }
    
    // We should never reach here:
    CGAL_error();
    return Result::default_result();
}

Result_type tgSegmentNetwork::_walk_from_face(segnetFaceHandle face, const segnetPoint& np, const segnetPoint& p, Halfedge_set& crossed_edges) const
{
    const segnetTraitsAdaptor* m_traits = static_cast<const segnetTraitsAdaptor*>(arr.geometry_traits());
    
    // Construct an x-monotone curve connecting the target point np
    // to the source point p and check which CCB intersects this segment.
    segnetXMonotoneCurve           seg = m_traits->construct_x_monotone_curve_2_object()(np, p);
    const bool                     p_is_left = (m_traits->compare_xy_2_object()(np, p) == CGAL::LARGER);
    
    Inner_ccb_const_iterator       inner_ccb_iter;
    Outer_ccb_const_iterator       outer_ccb_iter;
    const segnetHalfedgeHandle     invalid_he;
    segnetHalfedgeHandle           he;
    segnetFaceHandle               new_face;
    bool                           is_on_edge;
    bool                           is_target;
    bool                           cv_is_contained_in_seg;
    segnetVertexHandle             new_vertex;
    const segnetVertexHandle       invalid_vertex;
    
    do {
        // Check whether p lies inside the current face (including its holes):
        if (arr.topology_traits()->is_in_face(&(*face), p, NULL))
        {
            // We know that p is located inside the current face, and we check
            // whether it lies inside one of its holes (or on the boundary of
            // its holes).
            cv_is_contained_in_seg = false;
            new_face = face;
            for (inner_ccb_iter = face->inner_ccbs_begin(); inner_ccb_iter != face->inner_ccbs_end(); ++inner_ccb_iter) {
                he = _intersection_with_ccb(*inner_ccb_iter, seg, p, p_is_left, crossed_edges, is_on_edge, is_target, cv_is_contained_in_seg,new_vertex);
                if (he == invalid_he && cv_is_contained_in_seg) {
                    return _deal_with_curve_contained_in_segment(*inner_ccb_iter, p_is_left, p, crossed_edges);
                }
                if (he != invalid_he) {
                    // Check if the query point is located on a vertex or on an edge.
                    if (is_target) {
                        return Result::make_result(he->target());
                    } else if (is_on_edge) {
                        return Result::make_result(he); 
                    }

                    if (new_vertex != invalid_vertex) {
                        // if we got here it means that a closer vertex then np was found
                        return (_walk_from_vertex(new_vertex, p, true, crossed_edges));
                    }
                
                    // Otherwise, cross over he to the incident face of its twin.
                    if (face != he->twin()->face()) {
                        new_face = he->twin()->face();
                        break;
                    }
                }
            }
                    
            // Check if we found a new face (hole) containing p. If not, the current
            // face contains p.
            if (new_face == face) {
                return Result::make_result(face);
            }
                    
            // Continue from the new face (hole).
            face = new_face;
        } else {
            // We know that p is not located inside the current face. We therefore
            // look for an edge on its outer boundary that intersects seg.
            new_face = face;
            for (outer_ccb_iter = face->outer_ccbs_begin(); outer_ccb_iter != face->outer_ccbs_end(); ++outer_ccb_iter) {
                he = _intersection_with_ccb(*outer_ccb_iter, seg, p, p_is_left, crossed_edges, is_on_edge, is_target, cv_is_contained_in_seg,new_vertex);
                if (he == invalid_he && cv_is_contained_in_seg) {
                    return _deal_with_curve_contained_in_segment(*outer_ccb_iter, p_is_left, p, crossed_edges);
                }
                if (he != invalid_he) {
                    // Check if the query point is located on a vertex or on an edge.
                    if (is_target) {
                        return Result::make_result(he->target());
                    } else if (is_on_edge) {
                        return Result::make_result(he); 
                    }
                            
                    if (new_vertex != invalid_vertex) {
                        // if we got here it means that a closer vertex then np was found
                        return (_walk_from_vertex(new_vertex, p, true, crossed_edges));
                    }
                
                    // Otherwise, cross over he to the incident face of its twin.
                    if (face != he->twin()->face()) {
                        new_face = he->twin()->face();
                        break;
                    }
                }
            }
                    
            // Continue from the new face.
            CGAL_assertion(new_face != face);
            face = new_face;
        }
    } while (true);
    
    // We should never reach here:
    CGAL_error();
    return Result::default_result();
}