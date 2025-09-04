cmake_minimum_required(VERSION 3.25)
project(sem-stitcher LANGUAGES CXX C)


option(USE_IMGUI_VIEWER "Build GLFW/ImGui viewer" OFF)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/permissive->)


find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED
             COMPONENTS core imgproc features2d calib3d)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)


set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/semstitch.proto)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS   ${PROTO_FILE})
protobuf_generate_grpc_cpp(GRPC_SRCS  GRPC_HDRS ${PROTO_FILE})

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
    $<TARGET_OBJECTS:proto_objs>          
)

target_include_directories(semstitch PUBLIC
    core/include
    ${CMAKE_CURRENT_BINARY_DIR}           
)

target_link_libraries(semstitch
    PRIVATE
        OpenCV::opencv_core
        OpenCV::opencv_imgproc
        OpenCV::opencv_features2d
        OpenCV::opencv_calib3d
        Threads::Threads
        gRPC::grpc++ gRPC::grpc
        protobuf::libprotobuf
        artimagen                        
)

add_executable(sem-stitch-cli src/semstitch/cli/main.cpp)
target_link_libraries(sem-stitch-cli PRIVATE semstitch)

if(USE_IMGUI_VIEWER)
    find_package(glfw3 REQUIRED)
    find_package(GLEW   REQUIRED)
    add_executable(sem-stitch-viewer
        src/semstitch/gui/imgui/Viewer.cpp)
    target_include_directories(sem-stitch-viewer PRIVATE src/semstitch/gui/imgui)
    target_link_libraries(sem-stitch-viewer PRIVATE
        semstitch glfw GLEW::GLEW OpenGL::GL)
endif()

enable_testing()
