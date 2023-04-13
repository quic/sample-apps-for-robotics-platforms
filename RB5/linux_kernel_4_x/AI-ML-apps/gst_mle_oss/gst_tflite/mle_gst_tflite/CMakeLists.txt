# GStreamer plugin.
set(GST_MLE_TFLITE gstqtimletflite)

add_library(${GST_MLE_TFLITE} SHARED
  mle_tflite.cc
)

target_include_directories(${GST_MLE_TFLITE} PUBLIC
  ${SYSROOT_AARCH_LIBDIR}/glib-2.0/
  ${SYSROOT_AARCH_LIBDIR}/glib-2.0/include
  ${SYSROOT_INCDIR}/gstreamer-1.0
  ${SYSROOT_INCDIR}/glib-2.0
  ${SYSROOT_INCDIR}/qrb5165/glib-2.0
  ${SYSROOT_INCDIR}/qrb5165/gstreamer-1.0
  ${SYSROOT_INCDIR}/qrb5165  
  ${SYSROOT_INCDIR}/qrb5165/tflite
)

target_include_directories(${GST_MLE_TFLITE} PRIVATE
  ${CMAKE_SOURCE_DIR}
)

target_link_libraries(${GST_MLE_TFLITE} PRIVATE
  gstreamer-1.0
  gobject-2.0
  glib-2.0
  gstallocators-1.0
  gstreamer-1.0
  gstvideo-1.0
  gstbase-1.0
  qtimlmeta
  rt
  ${CMAKE_SOURCE_DIR}/mle_engine/libEngine_MLE.so
)

install(
  TARGETS ${GST_MLE_TFLITE}
  LIBRARY DESTINATION ${SYSROOT_AARCH_LIBDIR}/gstreamer-1.0
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              GROUP_EXECUTE GROUP_READ
)

