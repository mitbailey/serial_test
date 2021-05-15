/**
 * @file common.h
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021.05.14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_MESSAGE_SIZE 64

#ifndef eprintf
#define eprintf(str, ...)                                                    \
    {                                                                        \
    fprintf(stderr, "%s, %d: " str "\n", __func__, __LINE__, ##__VA_ARGS__); \
    fflush(stderr);                                                          \
    }                                                               
#endif // eprintf