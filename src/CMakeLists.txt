project(sfm_src)

add_executable(bundle_adjustment_global bundle_adjustment_global.cc)
target_link_libraries(bundle_adjustment_global
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  glog::glog
)