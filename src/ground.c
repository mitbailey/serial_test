/**
 * @file ground.c
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021.05.14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ground.h"

int main (void){
    eprintf("Ground is running.");

    // Set the hostname.
    char fname[20];
    char hstnm[20];

    gethostname(hstnm, 20);

    if (!strncmp(hstnm, "fl9361", 20))
        snprintf(fname, 20, "/dev/ttyPS0");
    else
        snprintf(fname, 20, "/dev/ttyUSB0");

    int fd = ground_init(fname, B9600);

    bool done_send = false;

    char message[] = "This is a test of the emergency alert broadcasting system.";
    eprintf("Wrote %ld bytes to serial.", ground_send(fd, message, &done_send));

    eprintf("Ground is finished.");
}

int ground_init(const char *sername, int baud){
    int fd = -1;
    // check device name
    if (sername == NULL)
    {
        eprintf("Serial device name pointer is null");
        goto ret;
    }
    // open device
    fd = open(sername, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd <= 0)
    {
        eprintf("Could not open %s", sername);
        goto ret;
    }
    // set attributes
    struct termios tty[1];
    if (tcgetattr(fd, tty))
    {
        eprintf("Error %d getting TTY attributes", errno);
        goto cleanup;
    }
    cfsetospeed(tty, baud);
    cfsetispeed(tty, baud);
    tty->c_cflag = (tty->c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty->c_iflag &= ~IGNBRK;          // disable break processing
    tty->c_lflag = 0;                 // no signaling chars, no echo,
                                      // no canonical processing
    tty->c_oflag = 0;                 // no remapping, no delays
    tty->c_cc[VMIN] = 1;              // read is blocking call
    tty->c_cc[VTIME] = GROUND_SLEEP_TIME; // 0.1 * GS_SLEEP_TIME seconds read timeout

    tty->c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty->c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                        // enable reading
    tty->c_cflag &= ~(PARENB | PARODD); // shut off parity
    // tty->c_cflag |= parity;
    tty->c_cflag &= ~CSTOPB;
    tty->c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, tty) != 0)
    {
        eprintf("Error %d from tcsetattr", errno);
        goto cleanup;
    }
    else
        goto ret; // success
cleanup:
    close(fd);
ret:
    return fd;
}

ssize_t ground_send(int fd, char message[], bool* done_send){
    ssize_t wr_sz = 0;
    ssize_t retval = 0;
    while (!done_send && wr_sz < MESSAGE_SIZE) {
        retval = read(fd, message + wr_sz, MESSAGE_SIZE - wr_sz);
        if (retval >= 0)
            wr_sz += retval;
    }
    return wr_sz;
}