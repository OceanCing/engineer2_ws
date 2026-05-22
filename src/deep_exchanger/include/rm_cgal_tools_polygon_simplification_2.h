//
// Created by kaxns on 3/12/22.
//

#pragma once

#include <boost/version.hpp>
#if BOOST_VERSION >= 105600 && (!defined(BOOST_GCC) || BOOST_GCC >= 40500)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>
#include <CGAL/Polyline_simplification_2/Stop_below_count_threshold.h>
namespace cgal
{
namespace polygon_simplification_2
{
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polyline_simplification_2::Stop_below_count_threshold Stop;
typedef CGAL::Polyline_simplification_2::Squared_distance_cost Cost;
}  // namespace polygon_simplification_2
}  // namespace cgal

#include <iostream>
#include <vector>
#include <opencv2/core/types.hpp>
namespace rm_cgal_tools_polygon_simplification
{
/**
 * @Ref https://doc.cgal.org/latest/Polyline_simplification_2/index.html
 * @Ref https://bjpcjp.github.io/pdfs/math/polygon-simplification-ADM.pdf
 */
template <typename T>
inline void approxPolyCGAL(const std::vector<cv::Point>& in_cv_pl, T& out_cv_pl, size_t thresh)
{
  cgal::polygon_simplification_2::Polygon_2 cgal_polygon{};
  for (auto& cv_pt : in_cv_pl)
  {
    cgal_polygon.push_back(cgal::polygon_simplification_2::Point_2(cv_pt.x, cv_pt.y));
  }
  cgal_polygon = CGAL::Polyline_simplification_2::simplify(cgal_polygon, cgal::polygon_simplification_2::Cost(),
                                                           cgal::polygon_simplification_2::Stop(thresh));
  for (size_t i = 0; i < thresh; ++i)
  {
    out_cv_pl[i].x = cgal_polygon[i].x();
    out_cv_pl[i].y = cgal_polygon[i].y();
  }
}
}  // namespace rm_cgal_tools_polygon_simplification

#else
template <typename T>
inline void approxPolyCGAL(const std::vector<cv::Point>& in_cv_pl, T& out_cv_pl, size_t thresh)
{
  static_assert((BOOST_VERSION >= 105600 && (!defined(BOOST_GCC) || BOOST_GCC >= 40500)),
                "Assert (BOOST_VERSION >= 105600 && (!defined(BOOST_GCC) || BOOST_GCC >= 40500)) failed.");
  std::terminate();
}
#endif
