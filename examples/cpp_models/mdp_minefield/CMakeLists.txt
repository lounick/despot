cmake_minimum_required(VERSION 2.8.3)

include_directories(
        src
)

add_executable("${PROJECT_NAME}_mdp_minefield"
  src/mdp_minefield.cpp
  src/main.cpp
)
target_link_libraries("${PROJECT_NAME}_mdp_minefield"
  "${PROJECT_NAME}"
)
install(TARGETS "${PROJECT_NAME}_mdp_minefield"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)
