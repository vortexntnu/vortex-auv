cmake_minimum_required(VERSION 2.8.12)

add_library(ViconDataStreamSDK_CPP
  Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP/DataStreamClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/CoreClient.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/ClientUtils.cpp
  Vicon/CrossMarket/DataStream/ViconDataStreamSDKCore/AxisMapping.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClientSDK/CGClient.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamClient.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/ViconCGStreamBayer.cpp
  Vicon/CrossMarket/DataStream/ViconCGStreamClient/CGStreamReaderWriter.cpp
)
target_include_directories(ViconDataStreamSDK_CPP
  PUBLIC Vicon/CrossMarket/DataStream
  PRIVATE Vicon/CrossMarket)
target_link_libraries(ViconDataStreamSDK_CPP ${Boost_LIBRARIES})
