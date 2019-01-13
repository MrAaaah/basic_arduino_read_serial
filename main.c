#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// from https://chrisheydrick.com/2012/06/17/how-to-read-serial-data-from-an-arduino-in-linux-with-c-part-3/
//
// Arduino on serial port /dev/ttyACM0 at 9600 bauds

pthread_t arduino_fetch_thread;

typedef struct 
{
    int sensor_value;

    pthread_mutex_t mutex;
} ArduinoThreadData;

static void * fn_arduino_get_data(void * p_data)
{
    ArduinoThreadData * data = (ArduinoThreadData *) p_data;

    const char * port = "/dev/ttyACM0";
    char buf[256];

    int fd;
    fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        printf("Failed to open serial port\n");
        return NULL;
    }

    struct termios toptions;

    tcgetattr(fd, &toptions);

    cfsetispeed(&toptions, B9600);
    cfsetospeed(&toptions, B9600);

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    toptions.c_oflag &= ~OPOST;

    toptions.c_cc[VMIN] = 5;
    toptions.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &toptions);

    tcflush(fd, TCIFLUSH);

    while (1)
    {
        usleep(50 * 1000);
        int n = read(fd, buf, 128);

        if (n == -1)
        {
            continue;
        }

        int digit_count = 0;
        for (int i = 0 ; i < n ; i++)
        {
            if (buf[i] != '\n')
                digit_count++;
        }

        if (digit_count > 0)
        {
            pthread_mutex_lock(&data->mutex);

            data->sensor_value = atoi(buf);

            pthread_mutex_unlock(&data->mutex);
        }
    }

    printf("Arduino thread ended\n");

    return NULL;
}

int main(int argc, char * argv[])
{
    ArduinoThreadData arduino_thread_data = { 0, PTHREAD_MUTEX_INITIALIZER };

    pthread_create(&arduino_fetch_thread, NULL, fn_arduino_get_data, (void *) &arduino_thread_data);

    while (1)
    {
        usleep(50 * 1000);

        pthread_mutex_lock(&arduino_thread_data.mutex);

        printf("-> %d\n", arduino_thread_data.sensor_value);

        pthread_mutex_unlock(&arduino_thread_data.mutex);
    }

    pthread_cancel(arduino_fetch_thread);


    printf("Exit\n");
    return 0;
}
