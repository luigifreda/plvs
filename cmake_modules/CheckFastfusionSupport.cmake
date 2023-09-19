
execute_process(COMMAND cat /proc/cpuinfo OUTPUT_VARIABLE CPU_INFO)

string(REGEX MATCHALL "avx2" AVX_STRING ${CPU_INFO})
list(LENGTH AVX_STRING AVX_STRING_LENGTH)
# message(STATUS "AVX_STRING: " ${AVX_STRING})
message(STATUS "Number of avx2 occurrences in /proc/cpuinfo: " ${AVX_STRING_LENGTH})

string(REGEX MATCHALL "sse3" SSE3_STRING ${CPU_INFO})
list(LENGTH SSE3_STRING SSE3_STRING_LENGTH)
# message(STATUS "SSE3_STRING: " ${SSE3_STRING})
message(STATUS "Number of sse3 occurrences in /proc/cpuinfo: " ${SSE3_STRING_LENGTH})

string(REGEX MATCHALL "sse4" SSE4_STRING ${CPU_INFO})
list(LENGTH SSE4_STRING SSE4_STRING_LENGTH)
# message(STATUS "SSE4_STRING: " ${SSE4_STRING})
message(STATUS "Number of sse4 occurrences in /proc/cpuinfo: " ${SSE4_STRING_LENGTH})

if( (${SSE3_STRING_LENGTH} GREATER 0) )
    set(HAVE_SSE3 true)
endif()

if( (${AVX_STRING_LENGTH} GREATER 0) OR (${SSE4_STRING_LENGTH} GREATER 0) )
    set(BUILD_FASTFUSION true)
endif()

