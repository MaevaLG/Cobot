# include directories
INCDIR+= -Isrc
INCDIR+= -Isrc/tcp
INCDIR+= -Isrc/detection_akaze
INCDIR+= -Isrc/traitement_texte


CXXFLAGS= -w -g -std=gnu++11 $(INCDIR) `pkg-config gtkmm-2.4 --cflags opencv --libs opencv `
CPP_SRC=$(shell find src -name "*.cpp" |grep -v \\/\\.)
OBJ=$(CPP_SRC:%.cpp=%.o)

all: $(OBJ)
	g++  -o programme $(OBJ) $(CXXFLAGS)
clean:
	rm -rf $(OBJ)
mrproper: clean
	rm -rf programme

