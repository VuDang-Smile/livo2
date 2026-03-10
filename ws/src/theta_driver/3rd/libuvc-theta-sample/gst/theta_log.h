/*
  Copyright 2020 K. Takeo. All rights reserved.

  THETA stream flow logging macros.
  Use grep '[THETA][INFO]' / '[THETA][WARN]' / '[THETA][ERROR]' to filter logs.
 */

#if !defined(__THETA_LOG_H__)
#define __THETA_LOG_H__

#include <stdio.h>

#define THETA_LOG_INFO(fmt, ...)    fprintf(stderr, "[THETA][INFO]  [%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#define THETA_LOG_WARNING(fmt, ...) fprintf(stderr, "[THETA][WARN]  [%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#define THETA_LOG_ERROR(fmt, ...)   fprintf(stderr, "[THETA][ERROR] [%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#endif /* __THETA_LOG_H__ */
