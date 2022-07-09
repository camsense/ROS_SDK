INCLUDE_DIRECTORIES(.)

SET(LIDAR_SRC hchead.cpp hclidar.cpp lidar.cpp ReadParsePackage.cpp HC_serial.cpp HcSDK.cpp)
ADD_LIBRARY(lidar_tof ${LIDAR_SRC})
IF (WIN32)
ELSE()
    TARGET_LINK_LIBRARIES(lidar_tof pthread)
ENDIF()
