add_executable(bug_0 bug_0.cpp)
target_link_libraries(bug_0 PRIVATE extern)
target_compile_options(bug_0 PRIVATE -Wall -Wextra -pedantic -Werror)

add_executable(bug_2 bug_2.cpp)
target_link_libraries(bug_2 PRIVATE extern)
target_compile_options(bug_2 PRIVATE -Wall -Wextra -pedantic -Werror)

add_executable(bug_tan bug_tan.cpp)
target_link_libraries(bug_tan PRIVATE extern)
target_compile_options(bug_tan PRIVATE -Wall -Wextra -pedantic -Werror)
