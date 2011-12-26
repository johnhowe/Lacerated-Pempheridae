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

#define true 1
#define false 0

#define START_BYTE          0xAA
#define ESCAPE_BYTE         0xBB
#define STOP_BYTE           0xCC

#define ESCAPED_START_BYTE  0x55
#define ESCAPED_ESCAPE_BYTE 0x44
#define ESCAPED_STOP_BYTE   0x33


#define STOP_PACKET_DELAY     15000 // microseconds to wait after sending a stop logging packet
#define TEST_PACKET_DELAY     25000 // microseconds to wait after sending a bench test packet
#define RPM_PACKET_DELAY       5000 // microseconds to wait between rpm setter packets

#define DISPLAY_HZ 10 // rate to update console output

#define BAUD_RATE 115200


#define BASETEETH_INDEX 4
#define TICKS_PER_EVENT_LOW_BYTE_INDEX   8
#define TICKS_PER_EVENT_HIGH_BYTE_INDEX  7
#define RPM_LOW_BYTE_INDEX              10
#define RPM_HIGH_BYTE_INDEX              9


#define microSecondsInOneMinute  60000000
#define tickSizeInFreeEMS               0.8

enum pattern {
        triangle, sinusoid, exponential
};

int minRPM = 120;
int maxRPM = 12000;
int startRPM = 120;
int duration = 1;
uint8_t baseTeeth = 12;

int fd;

void usleep(unsigned long usec)
{
        struct timespec reqtime;
        reqtime.tv_sec = 0;
        reqtime.tv_nsec = usec * 1000;
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

void sendPacket(uint8_t* rawPacket, int rawLength)
{
	uint8_t encodedPacket[(rawLength * 2) + 2];    // Worst case, 100% escaped bytes + start and stop
	int encodedLength = encodePacket(rawPacket, encodedPacket, rawLength);    // Actual length returned
	write(fd, encodedPacket, encodedLength);
}

void stopLogging(void)
{
	// Write to RAM description{flag,  payloadID, locationID,     offset,     length, data};
	uint8_t stop_streaming[] = { 0x00, 0x01, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
	sendPacket(stop_streaming, sizeof(stop_streaming));
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
void setupBenchTest(uint8_t eventsPerCycle, uint16_t ticksPerEvent)
{
	static uint8_t setupBenchTestPacket[] = { 0x00, 0x77, 0x77, 0x01, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	uint8_t lowByte = (uint8_t)(0xFF & ticksPerEvent);
	uint8_t highByte = (uint8_t)(ticksPerEvent >> 8);
	setupBenchTestPacket[TICKS_PER_EVENT_LOW_BYTE_INDEX] = lowByte;
	setupBenchTestPacket[TICKS_PER_EVENT_HIGH_BYTE_INDEX] = highByte;
	setupBenchTestPacket[BASETEETH_INDEX] = eventsPerCycle;

        sendPacket(setupBenchTestPacket, sizeof(setupBenchTestPacket));
}

void writeRPM(uint16_t rpmTicks)
{

	static uint8_t adjustRpmPacket[] = { 0x00, 0x01, 0x00, 0xF0, 0x01, 0x00, 0x1A, 0x00, 0x02, 0x00, 0x00 };

        uint8_t lowByte = (uint8_t)(0xFF & rpmTicks);
        uint8_t highByte = (uint8_t)(rpmTicks >> 8);
        adjustRpmPacket[RPM_LOW_BYTE_INDEX] = lowByte;
        adjustRpmPacket[RPM_HIGH_BYTE_INDEX] = highByte;

	sendPacket(adjustRpmPacket, sizeof(adjustRpmPacket));
}

uint16_t getTicksFromRPM(uint16_t RPM)
{
	return (uint16_t)((microSecondsInOneMinute / tickSizeInFreeEMS) / (RPM * baseTeeth));
}

void startSweep(void)
{
        enum pattern sweepShape = triangle;

        uint16_t RPM = minRPM;
        int rising = 1;
        switch(sweepShape) {
        //it should really be a percentage change, such that at say 100 rpm it changes by 1 rpm per second and at 200 rpm 2 per second and so on
        case(triangle):
        default:
                printf("Triangle wave:\nMin RPM: %d\nMax RPM: %d\n",minRPM, maxRPM);
                while (true) {
                        if (rising){
                                RPM++;
                                if (RPM >= maxRPM) {
                                        rising = false;
                                }
                        } else {
                                RPM--;
                                if (RPM <= minRPM) {
                                        rising = true;
                                }
                        }

                        static int dispCountdown = 0;
                        if (dispCountdown == 0) {
                                printf("\r %6d RPM",RPM);
                                fflush(stdout);
                                dispCountdown = 1000000/RPM_PACKET_DELAY/DISPLAY_HZ;
                        }
                        dispCountdown--;

                        writeRPM(getTicksFromRPM(RPM));
                        usleep(RPM_PACKET_DELAY);
                }
                break;
        }
}

void parseArg(char *arg)
{
	char *p = arg + 1;

	switch (*p) {
                /* Help */
	case 'h':
		printf("Help is on the way.\n");
		exit(1);
		break;

		/* Start RPM */
	case 's':
		startRPM = atoi(p + 1);
		break;

		/* Min RPM */
	case 'j':
		minRPM = atoi(p + 1);
		break;

		/* Max RPM */
	case 'k':
		maxRPM = atoi(p + 1);
		break;

		/* Pattern */
	case 'f':
		printf("Unsupported\n");
		break;

		/* Base teeth */
	case 't':
		baseTeeth = atoi(p + 1);
		break;

		/* Inter packet pause (microseconds) */
	case 'p':
		break;

		/* Duration of sweep (seconds) */
	case 'd':
		duration = atoi(p + 1);
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
        usleep(STOP_PACKET_DELAY);
	setupBenchTest(baseTeeth, getTicksFromRPM(startRPM));
        usleep(TEST_PACKET_DELAY);
	startSweep();

	tcsetattr(fd, TCSANOW, &oldtermios);
	close(fd);

	return 0;
}

