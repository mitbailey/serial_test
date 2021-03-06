/**
 * @file space.h
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021.05.14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#define SPACE_SLEEP_TIME 1

#include "common.h"

int space_init(const char *sername, int baud);

ssize_t space_recv(int fd, char message[], bool* done_recv);