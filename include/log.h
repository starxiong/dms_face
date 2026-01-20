#pragma once

#include "gflags/gflags.h"
// #include "glog/logging.h"
// #include "glog/raw_logging.h"
#include "defines.h"

FACEPOSE_BEGIN

// 调试信息
#ifndef ADEBUG
#define ADEBUG DLOG(INFO) << "[DEBUG] "
#endif // ADEBUG

// 一些不频繁但有关键的信息，比如说服务器地址和端口（调试信息除外）
#ifndef AINFO
// #define AINFO DLOG(INFO)
#define AINFO LOG(INFO)
#endif // AINFO

// 异常信息，但是系统能运行
#ifndef AWARN
#define AWARN LOG(WARNING)
#endif // AWARN

// 直接影响系统的错误出现
#ifndef AERROR
#define AERROR LOG(ERROR)
#endif // AERROR

// 程序运行不下去的信息（调用该接口会立即将堆栈信息打印并挂掉）
#ifndef AFATAL
#define AFATAL LOG(FATAL)
#endif // AFATAL

#define CHECK_LOG(condition)  \
      LOG_IF(FATAL, GOOGLE_PREDICT_BRANCH_NOT_TAKEN(!(condition))) \
             << "Check failed: " #condition " "
             
void InitLog(int argc, char* argv[]);

FACEPOSE_END