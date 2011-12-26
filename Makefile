# And a new sweeper was born! Swim little fishy, swim!

all:
	gcc --std=c99 -Wall -Werror -D _POSIX_C_SOURCE=200809L main.c -o pempheridae

debug:
	gcc --std=c99 -Wall -Werror -D DEBUG -D _POSIX_C_SOURCE=200809L main.c -o pempheridae


install:
	cp pempheridae ~/bin/

clean:
	rm pempheridae
