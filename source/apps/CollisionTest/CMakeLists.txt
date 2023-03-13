project(Tests_collision)

file(GLOB sources
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../helpers/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h #Important for unit testing to include h files here as well!
        ${CMAKE_CURRENT_SOURCE_DIR}/../helpers/include/*.h
        )

add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME}
        PUBLIC lenny::collision
        PUBLIC GTest::gtest_main
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../helpers/include
        )

target_compile_definitions(${PROJECT_NAME}
        PUBLIC LENNY_PROJECT_FOLDER=${LENNY_PROJECT_FOLDER}
        )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

include(GoogleTest)
gtest_add_tests(TARGET ${PROJECT_NAME}) #Problems with making 'gtest_discover_tests(${PROJECT_NAME})' work with ctest...