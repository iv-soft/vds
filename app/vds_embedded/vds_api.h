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
API void APICALL vds_done(APIENV api_void_ptr vds);
API api_string APICALL vds_last_error(APIENV api_void_ptr vds);

API api_string APICALL vds_start(APIENV api_void_ptr vds, int port, bool dev_network);

API void APICALL vds_set_root_folder(APIENV api_void_ptr vds, api_string root_folder);
API void APICALL vds_server_root(APIENV api_void_ptr vds, api_string login, api_string password);

//Login
API api_void_ptr APICALL vds_login(APIENV api_void_ptr vds, api_string login, api_string password);
API api_string APICALL vds_session_check(APIENV api_void_ptr vds_session);
API void APICALL vds_session_destroy(APIENV api_void_ptr vds_session);

API api_string APICALL vds_get_device_storages(APIENV api_void_ptr vds_session);
API api_string APICALL vds_prepare_device_storage(APIENV api_void_ptr vds_session);
API api_string APICALL vds_add_device_storage(APIENV api_void_ptr vds_session, api_string name, api_string local_path, int size);

API api_string APICALL vds_get_device_storage_path(APIENV api_void_ptr vds_session);
API uint64_t APICALL vds_get_device_storage_used(APIENV api_void_ptr vds_session);
API uint64_t APICALL vds_get_device_storage_size(APIENV api_void_ptr vds_session);

API api_string APICALL vds_set_device_storage_path(APIENV api_void_ptr vds_session, api_string new_path, uint64_t new_size);

API uint64_t APICALL vds_get_user_balance(APIENV api_void_ptr vds_session);


#ifdef __cplusplus
}
#endif

#endif //__VDS_EMBEDDED_VDS_API_H_
