include(FetchContent)

FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://gitee.com/zhaojiaoao/benchmark.git
    GIT_TAG        main
    CMAKE_ARGS 
       -DCMAKE_CXX_STANDARD=20 
)
FetchContent_MakeAvailable(googlebenchmark)