OBJ=obj/ #this doesn't get used cause we aren't smart enough
INC=include/
SRC=src/
LIB=lib/

CC=g++
CFLAGS=-c -Wall -I$(INC)

all: opengl

opengl: opengl_obj CoordSystemKinect_obj TrigLookup_obj
	$(CC) main_opengl.o CoordSystemKinect.o TrigLookup.o $(LIB)libfreenect.so -o opengl_exe -lfreenect -lGL -lGLU -lglut

main_opengl_obj:
	$(CC) $(CFLAGS) main_opengl.cpp

CoordSystemKinect_obj:
	$(CC) $(CFLAGS) $(SRC)CoordSystemKinect.cpp

TrigLookup_obj:
	$(CC) $(CFLAGS) $(SRC)TrigLookup.cpp

clean:
	rm -f *.o opengl_exe
