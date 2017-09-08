
cmake_minimum_required(VERSION 3.4.1)


add_library(native-lib
             SHARED
             src/main/cpp/native-lib.cpp )

find_library( log-lib
              log )

target_link_libraries(  native-lib
                       ${log-lib} )

set(PathToProject F:/Android/android-project/pose-estimation)
set(PathToOpenCV F:/Android/OpenCV-3.1.0-android-sdk/OpenCV-android-sdk)
include_directories(${PathToOpenCV}/sdk/native/jni/include)
add_library(pose_estimation SHARED src/main/cpp/pose_estimation.cpp)
add_library(lib_opencv SHARED IMPORTED)
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${PathToProject}/app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)
target_link_libraries(  pose_estimation
                        ${log-lib}
                        lib_opencv)