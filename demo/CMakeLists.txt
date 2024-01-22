add_executable(xvizMsgBridgeDemo xvizMsgBridgeDemo.cpp)
target_link_libraries(xvizMsgBridgeDemo 
                        xvizMsgBridge 
                        protoMessage
                        protobuf::libprotobuf
                        cppzmq cppzmq-static)

add_executable(auto_parking_demo auto_parking_demo.cpp)

target_link_libraries(auto_parking_demo 
                        auto_parking 
                        apollo_math
                        xvizMsgBridge 
                        protoMessage
                        protobuf::libprotobuf
                        cppzmq cppzmq-static)