#
# create some demos as ros nodes.
#

install(
  FILES demo_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_filter
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
