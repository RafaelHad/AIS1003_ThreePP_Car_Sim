cmake_minimum_required(VERSION 4.0)
project(live_07_10)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 619cdaacc2e7dc6ea709d8c7f482e2994af7349d
)
FetchContent_MakeAvailable(threepp)

add_executable(live_07_10 main.cpp)
target_link_libraries(live_07_10 PRIVATE threepp::threepp)
