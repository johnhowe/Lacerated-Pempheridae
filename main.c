/*
 * Lacerated Pempheridae
 *
 * A PC utility to sweep RPM with the MissingTooth hack benchtest
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

int minRPM = 50;
int maxRPM = 9000;
int startRPM = 50;
int dt = 1;
int duration = 1;
int teeth = 12;


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
                int rpm = calcRPM(i*dt);
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
        for (int i = 1; i < argc; i++) {
                char *arg = argv[i];

                if (arg[0] == '-') {
                        parseArg(arg);
                }
        }

        stopLogging();
        setupBenchTest();
        startSweep();

        return 0;
}


