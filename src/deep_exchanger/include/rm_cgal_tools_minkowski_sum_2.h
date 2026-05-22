#pragma once

#include <CGAL/Cartesian.h>
#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Arr_conic_traits_2.h>
#include <CGAL/Arrangement_2.h>
namespace cgal
{
namespace minkowski_sum_2
{
typedef CGAL::CORE_algebraic_number_traits Nt_traits;
typedef Nt_traits::Rational Rational;
typedef CGAL::Cartesian<Rational> Rational_kernel;
typedef Rational_kernel::Point_2 Rational_point;
typedef Rational_kernel::Segment_2 Rational_segment;
typedef Rational_kernel::Circle_2 Rational_circle;
typedef Nt_traits::Algebraic Algebraic;
typedef CGAL::Cartesian<Algebraic> Alg_kernel;

typedef CGAL::Arr_conic_traits_2<Rational_kernel, Alg_kernel, Nt_traits> Traits;
typedef Traits::Point_2 Point;
typedef Traits::Curve_2 Conic_arc;
typedef Traits::X_monotone_curve_2 X_monotone_conic_arc;
typedef CGAL::Arrangement_2<Traits> Arrangement;
}  // namespace minkowski_sum_2
}  // namespace cgal

#include <CGAL/Gps_traits_2.h>
#include <CGAL/offset_polygon_2.h>
namespace cgal
{
namespace minkowski_sum_2
{
typedef CGAL::Gps_traits_2<Traits> Gps_traits;
typedef Gps_traits::Polygon_2 Inset_polygon;
namespace rational
{
typedef CGAL::Polygon_2<Rational_kernel> Polygon_2;
typedef CGAL::Point_2<Rational_kernel> Point_2;
}  // namespace rational
}  // namespace minkowski_sum_2
}  // namespace cgal

#include <vector>
#include <opencv2/core/types.hpp>
namespace rm_cgal_tools_minkowski_sum_2
{
/**
 @brief Get the inset(inner offset) of a convex polygon.
 The idea is based on the Minkowski sum(difference), fortunately CGAL provides a complete implementation.
 @note Polygons must be convex, the case of concave polygons is not tested and guaranteed.
 @background Normal dilation or erosion cannot turn "sharp" polygons into rounded polygons, however Minkowski sum can do
 it. On the contrary, Minkowski difference can turn rounded polygons into "sharp" polygons, which can be understood as
 the inverse process of Minkowski sum.
 @ref https://algorist.com/problems/Minkowski_Sum.html
 @ref https://doc.cgal.org/latest/Minkowski_sum_2/index.html#title9
 */
template <typename OutputPolygonType, typename ElementType>
inline void insetConvexPolygon(std::vector<cv::Point>& cv_points, OutputPolygonType& output_polygon, int radius)
{
  cgal::minkowski_sum_2::rational::Polygon_2 shape{};
  for (auto& i : cv_points)
  {
    shape.push_back(cgal::minkowski_sum_2::rational::Point_2(i.x, i.y));
  }
  cgal::minkowski_sum_2::Traits traits;
  std::list<cgal::minkowski_sum_2::Inset_polygon> inset_polygons{};
  CGAL::inset_polygon_2(shape, radius, traits, std::back_inserter(inset_polygons));
  // std::cout << "The inset comprises " << inset_polygons.size() << " polygon(s)." << std::endl;
  if (inset_polygons.size() != 1)
    // As the input polygon is convex, inset_polygons.size() == 1 is necessary.
    return;
  std::list<cgal::minkowski_sum_2::Inset_polygon>::iterator it = inset_polygons.begin();
  for (auto i = it->curves_begin(); i != it->curves_end(); ++i)
  {
    // 2022.3.10.
    // Currently I don't know what to do with the awkward looking code below, its correctness is not proven, just
    // tested. In extreme cases such as large numbers, correctness cannot be guaranteed. Hope you can make it better.
    output_polygon.push_back(ElementType(i->target().x().approx().BigIntValue().intValue(),
                                         i->target().y().approx().BigIntValue().intValue()));
  }
}

}  // namespace rm_cgal_tools_minkowski_sum_2