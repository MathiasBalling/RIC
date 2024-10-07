find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV not found")
else()
  message(STATUS "OpenCV found via find_package")
endif()

add_executable(bug_0 bug_0.cpp)
target_link_libraries(bug_0 PRIVATE ${OpenCV_LIBS})
target_compile_options(bug_0 PRIVATE -Wall -Wextra -pedantic -Werror)

add_executable(bug_2 bug_2.cpp)
target_link_libraries(bug_2 PRIVATE ${OpenCV_LIBS})
target_compile_options(bug_2 PRIVATE -Wall -Wextra -pedantic -Werror)

add_executable(bug_tan bug_tan.cpp)
target_link_libraries(bug_tan PRIVATE ${OpenCV_LIBS})
target_compile_options(bug_tan PRIVATE -Wall -Wextra -pedantic -Werror)
