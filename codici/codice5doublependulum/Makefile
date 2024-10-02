
TARGET = bulletphysics.exe


CC = g++
CCFLAGS = -O3 -s -DNDEBUG


ifeq ($(OS),Windows_NT)
    BASEDIR = ../base
    INCLUDEDIRS = -I$(BASEDIR)/freeglut/include -I$(BASEDIR)/glew/include -I$(BASEDIR)/glm -I../include

    LIBDIRS = -L$(BASEDIR)/freeglut/lib -L$(BASEDIR)/glew/lib/Release/Win32 -L../lib

    LIBS = -lfreeglut -lopengl32 -lglew32 -lBulletDynamics -lBulletCollision -lLinearMath

else
    INCLUDEDIRS = -I../include
    LIBDIRS = -L../lib
    LIBS = -lglut -lGLEW -lGL
endif


OBJS = main.o utilities.o transform.o camera.o shaderclass.o myshaderclass.o light.o texture2D.o 


$(TARGET): $(OBJS)
	$(CC) $(CCFLAGS) $(OBJS) $(LIBDIRS) $(LIBS) -o $(TARGET)


main.o: main.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

utilities.o: utilities.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

transform.o: transform.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

camera.o: camera.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

shaderclass.o: shaderclass.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

myshaderclass.o: myshaderclass.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

light.o: light.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@

texture2D.o: texture2D.cpp
	$(CC) -c $(CCFLAGS) $(INCLUDEDIRS) $< -o $@


.PHONY: clean
clean:
	rm -f *.o *.exe
