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

#define BAUD_RATE 115200
int minRPM = 50;
int maxRPM = 9000;
int startRPM = 50;
int duration = 1;
int teeth = 12;

int fd;

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

void writeRPM(int rpm)
{
	printf("%d\n", rpm);

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
		teeth = atoi(p + 1);
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

