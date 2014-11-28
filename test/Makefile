CC = g++
CFLAGS  = -g -Wall -I../firmware -DDEBUG

default: main


main:  main.o mcdriver.o fixed.o
	$(CC) $(CFLAGS) -o main main.o mcdriver.o fixed.o

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

mcdriver.o: ../firmware/mcdriver.cpp
	$(CC) $(CFLAGS) -c ../firmware/mcdriver.cpp

fixed.o: ../firmware/fixed.cpp
	$(CC) $(CFLAGS) -c ../firmware/fixed.cpp

clean:
	$(RM) main *.o *~
