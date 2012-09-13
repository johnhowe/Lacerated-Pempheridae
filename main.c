/*
 * Lacerated Pempheridae
 *
 * A PC utility to sweep RPM with the MissingTooth hack benchtest
 *
 * @author John Howe
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
#include <time.h>

#define DEVICE "/dev/ttyUSB0"

#define true 1
#define false 0

#define START_BYTE          0xAA
#define ESCAPE_BYTE         0xBB
#define STOP_BYTE           0xCC

#define ESCAPED_START_BYTE  0x55
#define ESCAPED_ESCAPE_BYTE 0x44
#define ESCAPED_STOP_BYTE   0x33


#define STOP_PACKET_DELAY     15 // milliseconds to wait after sending a stop logging packet
#define TEST_PACKET_DELAY     25 // milliseconds to wait after sending a bench test packet
#define RPM_PACKET_DELAY       5 // milliseconds to wait between rpm setter packets

#define DISPLAY_HZ 15 // rate to update console output

#define BAUD_RATE 115200


#define BASETEETH_INDEX 4
#define TICKS_PER_EVENT_LOW_BYTE_INDEX   8
#define TICKS_PER_EVENT_HIGH_BYTE_INDEX  7
#define RPM_LOW_BYTE_INDEX              10
#define RPM_HIGH_BYTE_INDEX              9

#define TOO_MANY_ERRORS 20

#define microSecondsInOneMinute  60000000
#define tickSizeInFreeEMS               0.8

enum pattern {
        none, triangle, sinusoid, exponential, fileRead
};
enum pattern sweepShape = none;


char sweepFile[100];

int minRPM = 120;
int maxRPM = 12000;
uint8_t baseTeeth = 12;
int stepIncrement = 10;
int doRepeat = false;
int nMissingTeeth = 1;

int fd;
#ifdef DEBUG
FILE *dbgfp;
#endif

void msleep(unsigned long msec)
{
        struct timespec reqtime;
        reqtime.tv_sec = msec/1000;
        reqtime.tv_nsec = (msec%1000) * 1000000;
        nanosleep(&reqtime, NULL);
}

/*
 * Function to turn a packet structure into something serialisable for FreeEMS consumption.
 *
 * @note output array must be sized = (lengthOfInput * 2) + 2
 *
 * @return the encoded length, including start/stop bytes
 *
 * @author Fred Cooke
 */
int encodePacket(uint8_t* rawPacket, uint8_t* encoded, int rawLength)
{
	uint8_t checksum = 0;
	encoded[0] = START_BYTE;
	int outputIndex = 1;
	for (int inputIndex = 0; inputIndex < rawLength; inputIndex++) {
		checksum += rawPacket[inputIndex];
		switch (rawPacket[inputIndex]) {
		case START_BYTE: {
			encoded[outputIndex] = ESCAPE_BYTE;
			outputIndex++;
			encoded[outputIndex] = ESCAPED_START_BYTE;
			outputIndex++;
			break;
		}
		case ESCAPE_BYTE: {
			encoded[outputIndex] = ESCAPE_BYTE;
			outputIndex++;
			encoded[outputIndex] = ESCAPED_ESCAPE_BYTE;
			outputIndex++;
			break;
		}
		case STOP_BYTE: {
			encoded[outputIndex] = ESCAPE_BYTE;
			outputIndex++;
			encoded[outputIndex] = ESCAPED_STOP_BYTE;
			outputIndex++;
			break;
		}
		default: {
			encoded[outputIndex] = rawPacket[inputIndex];
			outputIndex++;
			break;
		}
		}
	}
	encoded[outputIndex] = checksum;
	outputIndex++;
	encoded[outputIndex] = STOP_BYTE;
	outputIndex++;
	return outputIndex;
}

int sendPacket(uint8_t* rawPacket, int rawLength)
{
        int txSuccess = true;
        // Read and discard any incoming data
        int ret = 0;
        uint8_t buf;
        do {
                ret = read(fd, &buf, 1);
        } while (ret > 0);

	uint8_t encodedPacket[(rawLength * 2) + 2];    // Worst case, 100% escaped bytes + start and stop
	int encodedLength = encodePacket(rawPacket, encodedPacket, rawLength);    // Actual length returned
#ifndef DEBUG
        if (write(fd, encodedPacket, encodedLength) == encodedLength) {
                txSuccess = true;
                tcdrain(fd);
        } else {
                txSuccess = false;
                fprintf(stderr, "Write to " DEVICE " failed.\n");
                msleep(RPM_PACKET_DELAY*10);
        }
#else
        fwrite(encodePacket, sizeof(uint8_t), encodedLength, dbgfp);
        fflush(dbgfp);
#endif
        return txSuccess;
}

int stopLogging(void)
{
	// Write to RAM description{flag,  payloadID, locationID,     offset,     length, data};
	uint8_t stop_streaming[] = { 0x00, 0x01, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
	return sendPacket(stop_streaming, sizeof(stop_streaming));
}

/**
 *
 * Creates and writes a special bench test initiate packet.
 *
 * Packet content description:
 *
 * 0x00 flags - correct
 * 0x7777 payload ID - correct
 * 0x01 mode - correct
 * 0x0C events per cycle / base teeth = 12/0x0C for default, 36/0x24 for common
 * 0xFFFF cycles, max this out
 * 0x???? ticks per event / effective rpm - set this dynamically, this is the initial value only
 * 00 00 00 00 00 00 - zeros for ease of manual checksum verification
 * 0x0003 This triggers the special mode
 * 0x0000 irrelevant as long as it's not 0x3, zeros easy to count
 * 0x0000 irrelevant as long as it's not 0x3, zeros easy to count
 * 0x0000 irrelevant as long as it's not 0x3, zeros easy to count
 * 0x0000 irrelevant as long as it's not 0x3, zeros easy to count
 * 0x0000 irrelevant as long as it's not 0x3, zeros easy to count
 */
int setupBenchTest(uint8_t eventsPerCycle, uint16_t ticksPerEvent)
{
        const int missingToothOffset = 16;
	static uint8_t setupBenchTestPacket[] = { 0x00, 0x77, 0x77, 0x01, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        if (nMissingTeeth > 5 || nMissingTeeth < 0) {
                nMissingTeeth = 1;
                fprintf(stderr, "Using 1 missing tooth. \n");
        }
        setupBenchTestPacket[missingToothOffset + 2*nMissingTeeth] = 0x03;

	uint8_t lowByte = (uint8_t)(0xFF & ticksPerEvent);
	uint8_t highByte = (uint8_t)(ticksPerEvent >> 8);
	setupBenchTestPacket[TICKS_PER_EVENT_LOW_BYTE_INDEX] = lowByte;
	setupBenchTestPacket[TICKS_PER_EVENT_HIGH_BYTE_INDEX] = highByte;
	setupBenchTestPacket[BASETEETH_INDEX] = eventsPerCycle;

        return sendPacket(setupBenchTestPacket, sizeof(setupBenchTestPacket));
}

int writeRPM(uint16_t rpmTicks)
{
        static int initialised = false;
        if (!initialised) {
                msleep(STOP_PACKET_DELAY);
                setupBenchTest(baseTeeth, rpmTicks);
                msleep(TEST_PACKET_DELAY);
                initialised = true;

                return true;
        } else {

                static uint8_t adjustRpmPacket[] = { 0x00, 0x01, 0x00, 0xF0, 0x01, 0x00, 0x1A, 0x00, 0x02, 0x00, 0x00 };

                uint8_t lowByte = (uint8_t)(0xFF & rpmTicks);
                uint8_t highByte = (uint8_t)(rpmTicks >> 8);
                adjustRpmPacket[RPM_LOW_BYTE_INDEX] = lowByte;
                adjustRpmPacket[RPM_HIGH_BYTE_INDEX] = highByte;

                return sendPacket(adjustRpmPacket, sizeof(adjustRpmPacket));
        }
}

uint16_t getTicksFromRPM(uint16_t RPM)
{
        uint16_t ticks = (uint16_t)((microSecondsInOneMinute / tickSizeInFreeEMS) / (RPM * baseTeeth));
        if (ticks < 200) {
                ticks = 200;
        }
	return ticks;
}

void dispRPM(int RPM)
{
        static int dispCountdown = 0;
        if (dispCountdown == 0 || RPM == minRPM || RPM == maxRPM) {
                printf("\r %6d RPM",RPM);
                fflush(stdout);
                dispCountdown = 1000/RPM_PACKET_DELAY/DISPLAY_HZ;
        }
        dispCountdown--;
}

int startFileSweep(void)
{
        printf("File sweep:\n");
        struct trap {
                unsigned long timestamp;
                unsigned int rpm;
        };

        do {
                unsigned long currentTime = 0;
                int RPM = 0;

                FILE *fp;
                fp = fopen(sweepFile, "r");
                if (fp == NULL) {
                        fprintf(stderr, "Can't open input file %s.\n", sweepFile);
                        return false;
                }

                struct trap pastPoint, futurePoint;

                pastPoint.timestamp = 0;
                pastPoint.rpm = 0;

                while (fscanf(fp, "%ld %d", &(futurePoint.timestamp), &(futurePoint.rpm)) != EOF){
                        while (currentTime <= futurePoint.timestamp){
                                if ((futurePoint.timestamp - pastPoint.timestamp) == 0){
                                        RPM = futurePoint.rpm;
                                } else {
                                        long n1 = (currentTime - pastPoint.timestamp)*futurePoint.rpm;
                                        long n2 = (currentTime - pastPoint.timestamp)*pastPoint.rpm;
                                        long d = futurePoint.timestamp - pastPoint.timestamp;
                                        RPM = (int)(pastPoint.rpm + (double)((n1 - n2)/(d)));
                                }

                                dispRPM(RPM);
                                if (writeRPM(getTicksFromRPM(RPM)) == false) {
                                        static int errCount = 0;
                                        if (++errCount > TOO_MANY_ERRORS) {
                                                fclose(fp);
                                                fprintf(stderr, "Too many errors.\n");
                                                return false;
                                        }
                                }

                                msleep(RPM_PACKET_DELAY);
                                currentTime += RPM_PACKET_DELAY;
                        }
                        pastPoint = futurePoint;
                }

                fclose(fp);
        } while (doRepeat);

        return true;
}

int startTriangleSweep(void)
{
        //"it should really be a percentage change, such that at say 100 rpm it
        //changes by 1 rpm per second and at 200 rpm 2 per second and so on"
        uint16_t RPM = minRPM;
        int rising = true;

        printf("Triangle wave:\nMin RPM: %d\nMax RPM: %d\nIncrement size: %d\n",minRPM, maxRPM, stepIncrement);
        while (true) {
                if (rising){
                        RPM+=stepIncrement;
                        if (RPM >= maxRPM) {
                                rising = false;
                        }
                } else {
                        RPM-=stepIncrement;
                        if (RPM <= minRPM) {
                                if (doRepeat) {
                                        rising = true;
                                } else {
                                        break;
                                }
                        }
                }

                dispRPM(RPM);

                if (writeRPM(getTicksFromRPM(RPM)) == false) {
                        static int errCount = 0;
                        if (++errCount > TOO_MANY_ERRORS) {
                                fprintf(stderr, "Too many errors.\n");
                                return false;
                        }
                }

                msleep(RPM_PACKET_DELAY);
        }
        dispRPM(RPM);
        return true;
}


void startSweep(void)
{
        switch(sweepShape) {
        case(fileRead):
                startFileSweep();
                break;
        case(triangle):
                startTriangleSweep();
                break;
        case(none):
        default:
                printf("No pattern specified, defaulting to triangle.\n");
                startTriangleSweep();
                break;
        }
}

void parseArg(char *arg)
{
	char *p = arg + 1;

	switch (*p) {
                /* Help */
	case 'h':
		printf("Help?\n");
		printf("\n");
		printf("-j Minimum RPM\n");
		printf("-k Maximum RPM\n");
		printf("-p Inter packet pause (microseconds)\n");
		printf("-t Base teeth\n");
		printf("-m Number of missing teeth\n");
		printf("-f Sweep file location\n");
		printf("-r Repeat the waveform\n");
		printf("-i RPM increment at each step in triangle mode\n");
		exit(1);
		break;

		/* Min RPM */
	case 'j':
		minRPM = atoi(p+2);
		break;

		/* Max RPM */
	case 'k':
		maxRPM = atoi(p+2);
		break;

                /* RPM increment at each step in triangle mode */
        case 'i':
                stepIncrement = atoi(p+2);
                break;

		/* Base teeth */
	case 't':
		baseTeeth = atoi(p+2);
		break;

		/* Inter packet pause (microseconds) */
	case 'p':
		break;

                /* Sweep file */
        case 'f':
                strcpy(sweepFile, p+2);
                sweepShape = fileRead;
                break;

                /* Repeat the waveform */
        case 'r':
                doRepeat = true;
                break;

                /* Number of missing teeth */
        case 'm':
                nMissingTeeth = atoi(p+2);
                break;

	default:
		fprintf(stderr, "Bad argument '%s'\n", arg);
		exit(1);
	}
}

int main(int argc, char **argv)
{

#ifdef DEBUG
        dbgfp = fopen("/tmp/Pempheridae", "w");
        if (dbgfp == NULL) {
                fprintf(stderr, "Can't open /tmp/Pempheridae file.\n");
                return -1;
        }
#endif

        struct termios oldtermios, termopts;

	for (int i = 1; i < argc; i++) {
		char *arg = argv[i];

		if (arg[0] == '-') {
			parseArg(arg);
		}
	}

	fd = open(DEVICE, O_RDWR | O_NONBLOCK);
	tcgetattr(fd, &oldtermios);

	/* 8O1 */
	memset(&termopts, 0, sizeof(termopts));
	cfsetispeed(&termopts, BAUD_RATE);
	cfsetospeed(&termopts, BAUD_RATE);
	termopts.c_cflag |= PARENB;
	termopts.c_cflag |= PARODD;
	termopts.c_cflag &= ~CSTOPB;
	termopts.c_cflag &= ~CSIZE;
	termopts.c_cflag |= CS8;
	if (tcsetattr(fd, TCSANOW, &termopts) < 0) {
		perror("Unable to set terminal parameters");
#ifndef DEBUG
		tcsetattr(fd, TCSANOW, &oldtermios);
		close(fd);
		return -1;
#endif
	}

	stopLogging();
	startSweep();

	tcsetattr(fd, TCSANOW, &oldtermios);
	close(fd);
#ifdef DEBUG
        fclose(dbgfp);
#endif

        printf("\n");
	return 0;
}

