#ifndef RRTS_COMMON_LOG_H
#define RRTS_COMMON_LOG_H

#define NOTICE(text) {               \
  static bool flag = true;           \
  if(flag) {                         \
    ROS_INFO(text);                  \
    flag = false;                    \
  }                                  \
}                                    \

#endif  // RRTS_COMMON_LOG_H
