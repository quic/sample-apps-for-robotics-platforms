set(GST_MLE_LIBRARY Engine_MLE)

list(APPEND SOURCE_FILES "ml_engine_impl.cc")

add_definitions(-DSNPE_ENABLE)
list(APPEND SOURCE_FILES "snpe_base.cc")
list(APPEND SOURCE_FILES "snpe_detection.cc")
list(APPEND SOURCE_FILES "snpe_yolodetection.cc")

add_library(${GST_MLE_LIBRARY} SHARED
  ${SOURCE_FILES}
)

target_include_directories(${GST_MLE_LIBRARY} PUBLIC
  ${GST_INCLUDE_DIRS}
  ${SYSROOT_INCDIR}/ion_headers
)

target_include_directories(${GST_MLE_LIBRARY} PRIVATE
  ${KERNEL_BUILDDIR}/usr/include
  ${SYSROOT_INCDIR}/ion_headers
)

target_link_libraries(${GST_MLE_LIBRARY} PRIVATE
#  log
#  dl
#  cutils
  fastcvopt
)

target_link_libraries(${GST_MLE_LIBRARY} PRIVATE SNPE)

install(
  TARGETS ${GST_MLE_LIBRARY}
  LIBRARY DESTINATION ${SYSROOT_LIBDIR} 
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)
