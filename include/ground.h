/**
 * @file ground.h
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021.05.14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#define GROUND_SLEEP_TIME 1

#include "common.h"

int ground_init(const char *sername, int baud);

ssize_t ground_send(int fd, char message[], bool* done_send);