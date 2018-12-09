#ifndef __VDS_EMBEDDED_VDS_API_H_
#define __VDS_EMBEDDED_VDS_API_H_

/*
Copyright (c) 2017, Vadim Malyshev, lboss75@gmail.com
All rights reserved
*/

#ifdef __ANDROID__

#include <jni.h>
#define API JNIEXPORT
#define APICALL JNICALL
#define APIENV_ JNIEnv *env
#define APIENV JNIEnv *env,

typedef jstring api_string;
typedef jlong api_void_ptr;

#define api_return_string(str) env->NewStringUTF(str)

#define api_string_argument(local_name, paramenter_name)\
    std::string local_name;\
    {\
        const char * buf = env->GetStringUTFChars(paramenter_name, 0);\
        const jint length = env->GetStringLength(paramenter_name);\
        local_name = std::string(buf, length);\
        env->ReleaseStringUTFChars(paramenter_name, buf);\
    }

#elif _WIN32

#define API __declspec(dllexport)

#define APICALL __stdcall
#define APIENV_
#define APIENV

typedef const char * api_string;
typedef void * api_void_ptr;

#define api_return_string(str) (str)

#define api_string_argument(local_name, paramenter_name)\
    std::string local_name = paramenter_name;

#else

#define API __attribute__ ((visibility("default")))
#define APICALL
#define APIENV_
#define APIENV

typedef const char * api_string;
typedef void * api_void_ptr;

#define api_return_string(str) (str)

#define api_string_argument(local_name, paramenter_name)\
    std::string local_name = paramenter_name;

#endif


#ifdef __cplusplus
extern "C" {
#endif

API api_void_ptr APICALL vds_init(APIENV_);
API void APICALL vds_done(APIENV void * vds);
API api_string APICALL vds_last_error(APIENV api_void_ptr vds);

//Tasks
API api_string APICALL vds_task_state(APIENV api_void_ptr task);
API void APICALL vds_task_cancel(APIENV api_void_ptr task);
API void APICALL vds_task_destroy(APIENV api_void_ptr task);

//Login
API api_void_ptr APICALL vds_login_begin(APIENV api_void_ptr vds, api_string login, api_string password);

#ifdef __cplusplus
}
#endif

#endif //__VDS_EMBEDDED_VDS_API_H_
