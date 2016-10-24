cmake_minimum_required(VERSION 2.8.3)

include_directories(
        src
        ../mdp_minefield/src
)

add_executable("${PROJECT_NAME}_cooperative_minefield"
        #src/base/base_cooperative_minefield.cpp
        ../mdp_minefield/src/mdp_minefield.cpp
        src/cooperative_minefield/cooperative_minefield.cpp
        src/cooperative_minefield/main.cpp
        )
target_link_libraries("${PROJECT_NAME}_cooperative_minefield"
        "${PROJECT_NAME}"
        )

install(TARGETS "${PROJECT_NAME}_cooperative_minefield"
        RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
        )
