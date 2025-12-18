include(FetchContent)

FetchContent_Declare(
    asio 
    GIT_REPOSITORY https://gitee.com/hengke/asio.git
    GIT_TAG asio-1-31-0
)
FetchContent_MakeAvailable(asio)

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
target_compile_definitions(asio INTERFACE ASIO_STANDALONE)

install(
    DIRECTORY ${asio_SOURCE_DIR}/asio/include
    DESTINATION ${CMAKE_SOURCE_DIR}/3th/include/asio
)

