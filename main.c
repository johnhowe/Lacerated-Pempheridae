/*
 * Lacerated Pempheridae
 *
 * A PC utility to sweep RPM with the MissingTooth hack benchtest
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

#define BAUD_RATE 115200
int minRPM = 50;
int maxRPM = 9000;
int startRPM = 50;
int duration = 1;
int teeth = 12;

int fd;

void stopLogging(void)
{

}

void setupBenchTest(void)
{

}

void writeRPM(int rpm)
{
        printf("%d\n",rpm);


}

int calcRPM(int time)
{

        int rpm;
        rpm = 100*(time);
        return rpm;
}

void startSweep(void)
{
        int count = duration;
        for (int i = 0; i < count; i++) {
                int rpm = calcRPM(i);
                writeRPM(rpm);
                // sleep?
        }
}

void parseArg(char *arg)
{
        char *p = arg+1;

        switch (*p) {
                /* Help */
                case 'h':
                        printf("Help is on the way.\n");
                        exit(1);
                        break;

                /* Start RPM */
                case 's':
                        startRPM = atoi(p+1);
                        break;

                /* Min RPM */
                case 'j':
                        minRPM = atoi(p+1);
                        break;

                /* Max RPM */
                case 'k':
                        maxRPM = atoi(p+1);
                        break;

                /* Pattern */
                case 'f':
                        printf("Unsupported\n");
                        break;

                /* Base teeth */
                case 't':
                        teeth = atoi(p+1);
                        break;

                /* Inter packet pause (microseconds) */
                case 'p':
                        break;

                /* Duration of sweep (seconds) */
                case 'd':
                        duration = atoi(p+1);
                        break;

                default:
                        fprintf(stderr, "Bad argument '%s'\n", arg);
                        exit(1);
        }
}

int main(int argc, char **argv)
{
        struct termios oldtermios, termopts;

        for (int i = 1; i < argc; i++) {
                char *arg = argv[i];

                if (arg[0] == '-') {
                        parseArg(arg);
                }
        }

        fd = open("/dev/ttyUSB0", O_RDWR);
        tcgetattr(fd,&oldtermios);

        /* 8N1 */
        memset(&termopts, 0, sizeof(termopts));
        cfsetispeed(&termopts, BAUD_RATE);
        cfsetospeed(&termopts, BAUD_RATE);
        termopts.c_cflag &= ~PARENB;
        termopts.c_cflag &= ~CSTOPB;
        termopts.c_cflag &= ~CSIZE;
        termopts.c_cflag |= CS8;
        if (tcsetattr(fd,TCSANOW,&termopts)< 0) {
                perror("Die");

                tcsetattr(fd,TCSANOW,&oldtermios);
                close(fd);

                return -1;
        }

        stopLogging();
        setupBenchTest();
        startSweep();

        tcsetattr(fd,TCSANOW,&oldtermios);
        close(fd);

        return 0;
}


