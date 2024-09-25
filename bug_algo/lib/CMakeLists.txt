find_package(fmt)
find_package(OpenCV)
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV not found")
else()
  message(STATUS "OpenCV found via find_package")
endif()

if(NOT fmt_FOUND)
  message(STATUS "fmt not found")
else()
  message(STATUS "fmt found via find_package")
endif()

add_library(extern INTERFACE)
target_link_libraries(extern INTERFACE
fmt::fmt
${OpenCV_LIBS}
)
