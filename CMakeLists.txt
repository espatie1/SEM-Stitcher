cmake_minimum_required(VERSION 3.25)
project(sem-stitcher LANGUAGES CXX C)


option(USE_IMGUI_VIEWER "Build GLFW/ImGui viewer" OFF)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/permissive->)


find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED
             COMPONENTS core imgproc features2d calib3d imgcodecs highgui)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/semstitch.proto)

# ---------- protobuf (messages) ----------
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})

# ---------- gRPC (services) --------------
set(GRPC_SRCS ${CMAKE_CURRENT_BINARY_DIR}/semstitch.grpc.pb.cc)
set(GRPC_HDRS ${CMAKE_CURRENT_BINARY_DIR}/semstitch.grpc.pb.h)

add_custom_command(
  OUTPUT  ${GRPC_SRCS} ${GRPC_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
          --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
          --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
          -I ${CMAKE_CURRENT_SOURCE_DIR}/proto
          ${PROTO_FILE}
  DEPENDS ${PROTO_FILE}
  COMMENT "Generate gRPC sources"
)

add_library(proto_objs OBJECT ${PROTO_SRCS} ${PROTO_HDRS}
                              ${GRPC_SRCS}  ${GRPC_HDRS})
target_link_libraries(proto_objs PUBLIC gRPC::grpc++ protobuf::libprotobuf)


add_subdirectory(third_party/artimagen)


add_library(semstitch STATIC
    src/semstitch/core/Stitcher.cpp
    src/semstitch/core/BackendFactory.cpp
    src/semstitch/backend/cpu/CpuBackend.cpp
    src/semstitch/io/GrpcServer.cpp
    src/semstitch/io/GrpcClient.cpp
    src/semstitch/io/ArtimagenSource.cpp  
    src/semstitch/io/Recorder.cpp
    src/semstitch/io/ViewportScannerSource.cpp
    src/semstitch/align/PhaseCorrelator.cpp
    src/semstitch/align/Refiner.cpp
    src/semstitch/align/GridAlign.cpp
    src/semstitch/align/GlobalSolver.cpp
    src/semstitch/compose/Mosaic.cpp
    $<TARGET_OBJECTS:proto_objs>          
)

target_include_directories(semstitch PUBLIC
    src/semstitch/core/include
    src/semstitch/backend
    src/semstitch/io
    ${CMAKE_CURRENT_BINARY_DIR}
    third_party/artimagen/src
    ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(semstitch
    PUBLIC
        opencv_core        
        opencv_imgproc
        opencv_features2d
        opencv_calib3d
        Threads::Threads
        gRPC::grpc++ gRPC::grpc
        protobuf::libprotobuf
        artimagen                        
)

add_executable(sem-stitch-cli
    src/semstitch/cli/main.cpp
    src/semstitch/cli/modes.hpp
    src/semstitch/cli/args.hpp
    src/semstitch/cli/jitter_buffer.hpp
    src/semstitch/cli/utils.cpp
    src/semstitch/cli/utils.hpp
    src/semstitch/cli/mode_simulate.cpp
    src/semstitch/cli/mode_receive.cpp
    src/semstitch/cli/mode_play.cpp
)
target_link_libraries(sem-stitch-cli PRIVATE semstitch ${OpenCV_LIBS})


if(USE_IMGUI_VIEWER)
    find_package(glfw3 REQUIRED)
    find_package(GLEW   REQUIRED)
    add_executable(sem-stitch-viewer
        src/semstitch/gui/imgui/Viewer.cpp)
    target_include_directories(sem-stitch-viewer PRIVATE src/semstitch/gui/imgui)
    target_link_libraries(sem-stitch-viewer PRIVATE
        semstitch glfw GLEW::GLEW OpenGL::GL)
endif()
