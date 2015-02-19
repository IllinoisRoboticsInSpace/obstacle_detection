OBJ=obj/ #this doesn't get used cause we aren't smart enough
INC=include/
SRC=src/
LIB=lib/

CC=g++
CFLAGS=-c -Wall -I$(INC)


all: obs_det_obj CoordSystemKinect_obj TrigLookup_obj
	$(CC) -lfreenect main.o CoordSystemKinect.o TrigLookup.o $(LIB)libfreenect.so -o obs_det

obs_det_obj:
	$(CC) $(CFLAGS) main.cpp

CoordSystemKinect_obj:
	$(CC) $(CFLAGS) $(SRC)CoordSystemKinect.cpp

TrigLookup_obj:
	$(CC) $(CFLAGS) $(SRC)TrigLookup.cpp

clean:
	rm -f *.o obs_det
