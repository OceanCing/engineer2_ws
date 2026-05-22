#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "AssemblyPost.h"

// Export the nodelet(s) contained in this library.
// The first argument must be the fully-qualified C++ type.
// The second argument must be nodelet::Nodelet.
PLUGINLIB_EXPORT_CLASS(vision_exchanger::AssemblyPost, nodelet::Nodelet)
