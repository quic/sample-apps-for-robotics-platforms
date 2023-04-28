# GStreamer plugin.
set(GST_MLE_SNPE gstqtimlesnpe)

add_library(${GST_MLE_SNPE} SHARED
  mle_snpe.cc
)

target_include_directories(${GST_MLE_SNPE} PUBLIC
  ${GST_INCLUDE_DIRS}
)

target_include_directories(${GST_MLE_SNPE} PRIVATE
  ${KERNEL_BUILDDIR}/usr/include
  ${SYSROOT_INCDIR}/ion_headers
  ${CMAKE_SOURCE_DIR}
)

target_link_libraries(${GST_MLE_SNPE} PRIVATE
  ${GST_LIBRARIES}
  ${GST_ALLOC_LIBRARIES}
  ${GST_VIDEO_LIBRARIES}
  qtimlmeta
  Engine_MLE
  gstqtivideobase
)

install(
  TARGETS ${GST_MLE_SNPE}
  LIBRARY DESTINATION ${SYSROOT_AARCH_LIBDIR}/gstreamer-1.0
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)
