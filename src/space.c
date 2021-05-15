/**
 * @file space.c
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021.05.14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "space.h"

int main (void){
    eprintf("Space is running.");

    // Set the hostname.
    char fname[20];
    char hstnm[20];

    gethostname(hstnm, 20);

    if (!strncmp(hstnm, "fl9361", 20))
        snprintf(fname, 20, "/dev/ttyPS0");
    else
        snprintf(fname, 20, "/dev/ttyUSB0");

    int fd = space_init(fname, B9600);

    bool done_recv = false;

    char message[MESSAGE_SIZE];
    memset(message, 0x0, MESSAGE_SIZE);
    eprintf("Read %ld bytes from serial.", space_recv(fd, message, &done_recv));
    eprintf("Message contents:");

    for (int i = 0; i < MESSAGE_SIZE; i++){
        printf("%x", message[i]);
    } printf("\n");

    for (int i = 0; i < MESSAGE_SIZE; i++){
        printf("%c", message[i]);
    } printf("\n");
    
    eprintf("Space is finished.");
}

int space_init(const char *sername, int baud){
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
    tty->c_cc[VTIME] = SPACE_SLEEP_TIME; // 0.1 * GS_SLEEP_TIME seconds read timeout

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

ssize_t space_recv(int fd, char message[], bool* done_recv){
    ssize_t rd_sz = 0;
    ssize_t retval = 0;
    while (!(*done_recv) && rd_sz < MESSAGE_SIZE) {
        retval = read(fd, message + rd_sz, MESSAGE_SIZE - rd_sz);
        if (retval >= 0)
            rd_sz += retval;
    }
    return rd_sz;
}
