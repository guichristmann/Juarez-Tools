TARGET = darwin_motions

INCLUDE_DIRS = -I../../include -I../../../Framework/include
CXXFLAGS += -O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lncurses -lrt -ljpeg

all: darwin.a $(TARGET)

darwin.a:
	make -C ../../build

$(TARGET): ../../lib/darwin.a
	$(CXX) -fPIC -shared -o $(TARGET).so $(TARGET).cpp ../../lib/darwin.a $(INCLUDE_DIRS) $(LIBS)
	cp $(TARGET).so /usr/lib/python2.7/dist-packages/

