add_executable(catch2Tests catch2Main.cpp)
target_sources(catch2Tests PRIVATE
        test_mimo.cpp
        test_mimo_struct.cpp
        test_siso.cpp)


target_link_libraries(catch2Tests
        es01_library)

target_include_directories(catch2Tests PRIVATE ../include)

target_compile_options(catch2Tests PRIVATE -Wa,-mbig-obj)