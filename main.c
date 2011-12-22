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

#define PACKET_DELAY 3000 // microseconds to wait between packets
#define BAUD_RATE 115200
int minRPM = 50;
int maxRPM = 9000;
int startRPM = 50;
int duration = 1;
uint8_t baseteeth = 12;

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
	encoded[0] = 0xAA;
	int outputIndex = 1;
	for (int inputIndex = 0; inputIndex < rawLength; inputIndex++) {
		checksum += rawPacket[inputIndex];
		switch (rawPacket[inputIndex]) {
		case 0xAA: {
			encoded[outputIndex] = 0xBB;
			outputIndex++;
			encoded[outputIndex] = 0x55;
			outputIndex++;
			break;
		}
		case 0xBB: {
			encoded[outputIndex] = 0xBB;
			outputIndex++;
			encoded[outputIndex] = 0x44;
			outputIndex++;
			break;
		}
		case 0xCC: {
			encoded[outputIndex] = 0xBB;
			outputIndex++;
			encoded[outputIndex] = 0x33;
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
	encoded[outputIndex] = 0xCC;
	outputIndex++;
	return outputIndex;
}

void sendPacket(uint8_t* rawPacket, int rawLength)
{
	uint8_t encodedPacket[(rawLength * 2) + 2];    // Worst case, 100% escaped bytes + start and stop
	int encodedLength = encodePacket(rawPacket, encodedPacket, rawLength);    // Actual length returned
	write(fd, encodedPacket, encodedLength);
        usleep(PACKET_DELAY);
}

void stopLogging(void)
{
	// Write to RAM description{flag,  payloadID, locationID,     offset,     length, data};
	uint8_t stop_streaming[] = { 0x00, 0x01, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
	sendPacket(stop_streaming, 10);
}

void setupBenchTest(void)
{

}

/**
 *
 * Creates and writes an RPM packet.
 *
 * rpmPacket as so:
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
void writeRPM(int rpm)
{
	static uint8_t rpmPacket[] = { 0x00, 0x77, 0x77, 0x01, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        const int packetLength = 27;

        rpmPacket[4] = baseteeth;

        //uint16_t ticksPerEvent = 60000000 / (baseteeth * toothperiod * 0.8);
        const int clockHz = 1000000; // Fred, help me out here ;)
        uint16_t ticksPerEvent = (clockHz * 60) / (rpm * baseteeth);

        rpmPacket[7] = ticksPerEvent;

        sendPacket(rpmPacket, packetLength);
}

int calcRPM(int time)
{

	int rpm;
	rpm = 100 * (time);
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
		baseteeth = atoi(p + 1);
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

	/* 8N1 */
	memset(&termopts, 0, sizeof(termopts));
	cfsetispeed(&termopts, BAUD_RATE);
	cfsetospeed(&termopts, BAUD_RATE);
	termopts.c_cflag &= ~PARENB;
	termopts.c_cflag &= ~CSTOPB;
	termopts.c_cflag &= ~CSIZE;
	termopts.c_cflag |= CS8;
	if (tcsetattr(fd, TCSANOW, &termopts) < 0) {
		perror("Die");

		tcsetattr(fd, TCSANOW, &oldtermios);
		close(fd);

		return -1;
	}

	stopLogging();
	setupBenchTest();
	startSweep();

	tcsetattr(fd, TCSANOW, &oldtermios);
	close(fd);

	return 0;
}

