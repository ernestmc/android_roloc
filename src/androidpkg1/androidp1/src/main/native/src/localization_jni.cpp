#include <android/log.h>
#include <ros/ros.h>

#include "localization_jni.h"

#include <robot_localization/ros_filter_types.h>

using namespace std;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "Native_Robot_Localization", msg, args);
    va_end(args);
}

inline string stdStringFromjString(JNIEnv* env, jstring java_string) {
    const char* tmp = env->GetStringUTFChars(java_string, NULL);
    string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

bool running;

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved)
{
    log("Library has been loaded");
    // Return the JNI version
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_LocalizationNativeNode_execute (JNIEnv *env, jobject obj, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName, jobjectArray remappingArguments)
{
    log("Native localization node started.");
    running = true;

    string master("__master:=" + stdStringFromjString(env, rosMasterUri));
    string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
    string node_name(stdStringFromjString(env, rosNodeName));

    log(master.c_str());
    log(hostname.c_str());

    // Parse remapping arguments
    log("Before getting size");
    jsize len = env->GetArrayLength(remappingArguments);
    log("After reading size");

    std::string ni = "localization_jni";

    int argc = 0;
    const int static_params = 4;
    char** argv  = new char* [static_params + len];
    argv[argc++] = const_cast<char*>(ni.c_str());
    argv[argc++] = const_cast<char*>(master.c_str());
    argv[argc++] = const_cast<char*>(hostname.c_str());

    //Lookout: ros::init modifies argv, so the references to JVM allocated strings must be kept in some other place to avoid "signal 11 (SIGSEGV), code 1 (SEGV_MAPERR), fault addr deadbaad"
    // when trying to free the wrong reference ( see https://github.com/ros/ros_comm/blob/indigo-devel/clients/roscpp/src/libros/init.cpp#L483 )
    char** refs  = new char* [len];
    for(int i=0; i<len; i++) {
       refs[i] = (char *)env->GetStringUTFChars((jstring)env->GetObjectArrayElement(remappingArguments, i), NULL);
       argv[argc] = refs[i];
       argc++;
    }

    log("Initiating ROS...");
    ros::init(argc, &argv[0], node_name.c_str());
    log("ROS intiated.");

    // Release JNI UTF characters
    for(int i=0; i<len; i++) {
        env->ReleaseStringUTFChars((jstring)env->GetObjectArrayElement(remappingArguments, i), refs[i]);
    }
    delete refs;
    delete argv;

    // Create and extended kallman filter node
    RobotLocalization::RosEkf ekf;

    // TODO: get the correct frequency
    //ros::Rate loop_rate(odom.getFrequency());
    ros::Rate loop_rate(30);

    while(ros::ok()) {
        try {
            ekf.run();
            loop_rate.sleep();
        } catch(...) {
            log("Exception catched in main loop.");
        }
    }

    log("Exiting from JNI call.");
}

JNIEXPORT void JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_LocalizationNativeNode_shutdown
  (JNIEnv *, jobject)
{
    log("Shutting down native node.");
    ros::shutdown();
    running = false;
}
