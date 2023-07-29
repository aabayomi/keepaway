# cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.2)
project(keepaway_player)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -D_Debug -D_Compress")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")


include_directories(/usr/include/eigen3)

set(SOURCE_FILES
        player/ActHandler.cc
        player/ActHandler.h
        player/BasicPlayer.cc
        player/BasicPlayer.h
        player/Connection.cc
        player/Connection.h
        player/data.h #
        player/DemoExtension.cc
        player/Formations.cc
        player/Formations.h
        player/GenericValues.cc
        player/GenericValues.h
        player/Geometry.cc
        player/Geometry.h
        player/Graph.cc #
        player/Graph.h  #
        player/HandCodedAgent.cc
        player/HandCodedAgent.h
        player/KeepawayPlayer.cc
        player/KeepawayPlayer.h
        player/Logger.cc
        player/Logger.h
        player/LoggerDraw.cc
        player/LoggerDraw.h
        player/Layers.h
        player/main.cc #
        player/Objects.cc
        player/Objects.h
        player/Parse.cc
        player/Parse.h
        player/PlayerSettings.cc
        player/PlayerSettings.h
        player/SayMsgDataTypes.h
        player/SayMsgDecoder.h
        player/SayMsgEncoder.h
        player/SayMsgFactory.h
        player/SayMsgIDs.h
        player/SayMsgTypes.h
        player/SenseHandler.cc
        player/SenseHandler.h
        player/ServerSettings.cc
        player/ServerSettings.h
        player/SMDPAgent.h
        player/SMDPAgent.cc
        player/prettyprint.h
        player/SoccerTypes.cc
        player/SoccerTypes.h
        player/tiles2.cc
        player/tiles2.h
        player/WorldModel.cc
        player/WorldModel.h
        player/WorldModelHighLevel.cc
        player/WorldModelKeepaway.cc
        player/WorldModelPredict.cc
        player/WorldModelSayMsgFactory.h
        player/WorldModelSayMsgTypes.cc
        player/WorldModelSayMsgTypes.h
        player/WorldModelUpdate.cc
        player/HierarchicalFSM.cc
        player/HierarchicalFSM.h
        player/ChoicePoint.h
        player/LinearSarsaLearner.cc
        player/LinearSarsaLearner.h
        player/gzstream.cc
        player/gzstream.h)

# find_package(Eigen3 REQUIRED)
add_executable(keepaway_player ${SOURCE_FILES})
target_link_libraries(keepaway_player pthread dl rt z)



# cmake_minimum_required(VERSION 3.2)

# set(CMAKE_CXX_STANDARD 17)

# project(keepaway_player)


# include_directories(/usr/include/eigen3)

# add_custom_target(copy-runtime-files ALL
#     COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources ${CMAKE_BINARY_DIR}/resources
#     DEPENDS ${MY_TARGET})

# set(SOURCE_FILES player/main.cc player/data.h player/Layers.h player/Graph.h)
# add_executable(out ${SOURCE_FILES})

# install(TARGETS out DESTINATION bin)

