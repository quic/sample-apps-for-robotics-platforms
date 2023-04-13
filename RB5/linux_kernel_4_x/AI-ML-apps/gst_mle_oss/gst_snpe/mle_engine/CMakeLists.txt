cmake_minimum_required(VERSION 3.8.2)
set(GST_MLE_LIBRARY Engine_MLE_SNPE)

list(APPEND SOURCE_FILES "ml_engine_impl.cc")

list(APPEND SOURCE_FILES "snpe_base.cc")
list(APPEND SOURCE_FILES "snpe_detection.cc")
list(APPEND SOURCE_FILES "snpe_single_ssd.cc")
list(APPEND SOURCE_FILES "snpe_segmentation.cc")

set (SNPE_PATH ${SNPE_LIBDIR}/libSNPE.so)

add_library(${GST_MLE_LIBRARY} SHARED
  ${SOURCE_FILES}
)

target_include_directories(${GST_MLE_LIBRARY} PUBLIC
  ${SYSROOT_AARCH_LIBDIR}/glib-2.0/
  ${SYSROOT_AARCH_LIBDIR}/glib-2.0/include
  ${SYSROOT_INCDIR}/gstreamer-1.0
  ${SYSROOT_INCDIR}/glib-2.0
  ${SYSROOT_INCDIR}/qrb5165/glib-2.0
  ${SYSROOT_INCDIR}/qrb5165/gstreamer-1.0
  ${SYSROOT_INCDIR}/qrb5165
  ${SYSROOT_INCDIR}/qrb5165/ion_headers
)

target_include_directories(${GST_MLE_LIBRARY} PRIVATE
  ${SNPE_INCDIR}
)

target_link_libraries(${GST_MLE_LIBRARY} PRIVATE
  ${SYSROOT_LIBDIR}/liblog.so.0
  ${SYSROOT_AARCH_LIBDIR}/libdl.so
  ${SYSROOT_LIBDIR}/libcutils.so.0
  ${SYSROOT_AARCH_LIBDIR}/libjsoncpp.so
  qtimlmeta
  fastcvopt
  ${SNPE_PATH}
  /lib/aarch64-linux-gnu/librt.so.1
  ${SYSROOT_LIBDIR}/libtensorflow-lite.so
)

install(
  TARGETS ${GST_MLE_LIBRARY}
  LIBRARY DESTINATION ${SYSROOT_LIBDIR}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              GROUP_EXECUTE GROUP_READ
)
