all:	mouth_force_reader

mouth_force_reader:	Makefile mouth_force_reader.c
	gcc -Wall -g -o mouth_force_reader mouth_force_reader.c -lm
