#include <jni.h>
#include <string>


extern "C"
jstring
Java_com_example_betty_pose_1estimation_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
