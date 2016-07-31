cmake_minimum_required(VERSION 3.5)
project(keepaway_player)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    ActHandler.cc
    ActHandler.h
    BasicPlayer.cc
    BasicPlayer.h
    Connection.cc
    Connection.h
    DemoExtension.cc
    Formations.cc
    Formations.h
    GenericValues.cc
    GenericValues.h
    Geometry.cc
    Geometry.h
    HandCodedAgent.cc
    HandCodedAgent.h
    KeepawayPlayer.cc
    KeepawayPlayer.h
    LinearSarsaAgent.cc
    LinearSarsaAgent.h
    Logger.cc
    Logger.h
    LoggerDraw.cc
    LoggerDraw.h
    main.cc
    Objects.cc
    Objects.h
    Parse.cc
    Parse.h
    PlayerSettings.cc
    PlayerSettings.h
    SayMsgDataTypes.h
    SayMsgDecoder.h
    SayMsgEncoder.h
    SayMsgFactory.h
    SayMsgIDs.h
    SayMsgTypes.h
    SenseHandler.cc
    SenseHandler.h
    ServerSettings.cc
    ServerSettings.h
    SMDPAgent.h
    SoccerTypes.cc
    SoccerTypes.h
    tiles2.cc
    tiles2.h
    WorldModel.cc
    WorldModel.h
    WorldModelHighLevel.cc
    WorldModelKeepaway.cc
    WorldModelPredict.cc
    WorldModelSayMsgFactory.h
    WorldModelSayMsgTypes.cc
    WorldModelSayMsgTypes.h
    WorldModelUpdate.cc)

add_executable(keepaway_player ${SOURCE_FILES})
target_link_libraries(keepaway_player pthread dl)
