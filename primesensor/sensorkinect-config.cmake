GET_FILENAME_COMPONENT(HERE "${CMAKE_CURRENT_LIST_FILE}" PATH)
GET_FILENAME_COMPONENT(INCLUDE_DIR ${HERE}/../../include ABSOLUTE)
include_directories(${INCLUDE_DIR})
include(${HERE}/sensorkinect-exports.cmake)
