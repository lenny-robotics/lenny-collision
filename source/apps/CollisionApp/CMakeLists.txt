# fetch lenny-gui-opengl
fetch_lenny_repository(gui-opengl master)

# project
project(CollisionApp)

file(GLOB sources
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helpers/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        )

add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME}
        PUBLIC lenny::collision
        PUBLIC lenny::gui
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../helpers/include
        )

target_compile_definitions(${PROJECT_NAME}
        PUBLIC LENNY_PROJECT_FOLDER=${LENNY_PROJECT_FOLDER}
        )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
