
set(XvizMsgBridgeLib 
                        xvizMsgBridge 
                        protoMessage
                        protobuf::libprotobuf
                        cppzmq cppzmq-static)

add_executable(xvizMsgBridgeDemo xvizMsgBridgeDemo.cpp)
target_link_libraries(xvizMsgBridgeDemo 
                        ${XvizMsgBridgeLib})

add_executable(auto_parking_demo auto_parking_demo.cpp)
target_link_libraries(auto_parking_demo 
                        auto_parking 
                        apollo_math
                        ${XvizMsgBridgeLib}
                        spdlog::spdlog
                        )

add_executable(parking_case_parser_demo parking_case_parser_demo.cpp)
target_link_libraries(parking_case_parser_demo 
                        ${XvizMsgBridgeLib})



add_executable(map_demo map_demo.cpp)
target_link_libraries(map_demo 
                        ${XvizMsgBridgeLib})