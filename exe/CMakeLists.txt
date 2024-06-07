add_executable(NRSimulator NRSimulator.cpp)
target_link_libraries(NRSimulator simulator)

set(DTN_SOURCE_FILES
    DTN/Tracking/CMT.cpp
    DTN/Tracking/common.cpp
    DTN/Tracking/Consensus.cpp
    DTN/Tracking/Tracker.cpp
    DTN/Tracking/Matcher.cpp
    DTN/Tracking/Fusion.cpp
    DTN/Tracking/fastcluster/fastcluster.cpp
    DTN/Detection/inference.cpp
    DTN/Controllers/CameraController.cpp
    DTN/Controllers/NavigationController.cpp
    DTN/Controllers/DroneController.cpp
    DTN/main.cpp)

add_executable(DTNSimulator ${DTN_SOURCE_FILES})
target_link_libraries(DTNSimulator simulator)