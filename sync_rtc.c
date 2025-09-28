#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        return 1;
    }

    int serial_fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        fprintf(stderr, "Error opening serial port %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        fprintf(stderr, "Error getting serial port attributes: %s\n", strerror(errno));
        close(serial_fd);
        return 1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;         // Clear data size bits
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VTIME] = 10;  // 1 second timeout
    tty.c_cc[VMIN] = 0;    // Non-blocking read

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error setting serial port attributes: %s\n", strerror(errno));
        close(serial_fd);
        return 1;
    }

    printf("Waiting for next second boundary\n");

    struct timeval tv;
    gettimeofday(&tv, NULL);

    int usec_to_wait = 1000000 - tv.tv_usec;
    usleep(usec_to_wait);

    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    char time_str[8];
    snprintf(time_str, sizeof(time_str), "%02d%02d%02d\n",
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    ssize_t bytes_written = write(serial_fd, time_str, strlen(time_str));
    if (bytes_written < 0) {
        fprintf(stderr, "Error writing to serial port: %s\n", strerror(errno));
        close(serial_fd);
        return 1;
    }

    printf("Sent: %s (%02d:%02d:%02d) to %s at second boundary\n",
           time_str, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, argv[1]);

    close(serial_fd);
    return 0;
}
