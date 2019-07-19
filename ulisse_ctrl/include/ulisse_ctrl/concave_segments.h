#ifndef PROVA_DISTANCE_CONCAVE_SEGMENTS_H
#define PROVA_DISTANCE_CONCAVE_SEGMENTS_H

#include <iostream>
#include <list>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <math.h>

#include <boost/numeric/conversion/bounds.hpp>
#include <boost/foreach.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::linestring<point_type> linestring_type;
typedef boost::geometry::model::multi_point<point_type> mpoint_t;

template<typename Point>

void lines_from_convex_points(Point const &p);

template<typename Point>

void discretize_concave_segments(Point const &p, Point const &prec, Point const &next);

#endif //PROVA_DISTANCE_CONCAVE_SEGMENTS_H
