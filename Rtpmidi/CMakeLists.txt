add_library(${RTPMIDI_LIB} src/rtpmidi.cpp)
target_include_directories(${RTPMIDI_LIB} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/rtpmidid/include) # Should be PRIVATE
target_include_directories(${RTPMIDI_LIB} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#target_link_directories(${RTPMIDI_LIB} INTERFACE ${CMAKE_SOURCE_DIR}/3rdparty/rtpmidid/lib)
target_link_libraries(${RTPMIDI_LIB} PUBLIC -lfmt -lpthread)
target_link_libraries(${RTPMIDI_LIB} PRIVATE rtpmidid -lavahi-common -lavahi-client)
