
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/report)

find_program(CLANG_TIDY "clang-tidy")
if(CLANG_TIDY)

    file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
    file(GLOB_RECURSE TESTS "${CMAKE_SOURCE_DIR}/test/*.cpp")

    # set(CMAKE_CXX_CLANG_TIDY  ${CLANG_TIDY}  -p ${CMAKE_BINARY_DIR} -checks=clang-analyzer-*,modernize-* --warnings-as-errors=*)
    add_custom_target(clang-tidy-check
        COMMAND ${CLANG_TIDY} ${SOURCES}
        -p=${CMAKE_BINARY_DIR}
        -checks=clang-analyzer-*,modernize-*,cppcoreguidelines-*,bugprone-*,readability-*,performance-*,memory-*,-modernize-use-trailing-return-type
        -header-filter='(${CMAKE_SOURCE_DIR}/src/.*|${CMAKE_SOURCE_DIR}/include/.*)' # 这两个文件夹里的头文件也会被扫瞄
        -export-fixes=${CMAKE_SOURCE_DIR}/report/fixes.yaml
       COMMENT "Generating Clang-Tidy report..."
    )
endif()

find_program(CPPCHECK "cppcheck")
if(CPPCHECK)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/cppcheck_cache)
    add_custom_target(cppcheck-check
        COMMAND ${CPPCHECK}
            --template=vs
            -i ${CMAKE_BINARY_DIR}/* 
            --project=${CMAKE_BINARY_DIR}/compile_commands.json
            --cppcheck-build-dir=${CMAKE_BINARY_DIR}/cppcheck_cache
            --enable=all
            --std=c++20
            --inline-suppr
            --inconclusive
            --suppress=missingIncludeSystem
            -j 2
            --xml
            --xml-version=2
            2> ${CMAKE_SOURCE_DIR}/report/cppcheck_report.xml
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR} 
        COMMENT "Running Cppcheck static analysis ..."
    )
endif()

add_custom_target(static-analysis
    DEPENDS clang-tidy-check cppcheck-check
    COMMENT "Running all static analyzers..."
)