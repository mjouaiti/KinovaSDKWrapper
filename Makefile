CXX = g++
CPPFLAGS =
CXXFLAGS = -std=c++11
SOURCES = Robot.cpp Robot-API.cpp Robot-control.cpp Robot-measure.cpp \
          Robot-move.cpp Robot-phy-parameters.cpp Robot-time.cpp Robot-utils.cpp
OBJECTS = $(SOURCES:.cpp=.o)
HEADERS = Kinova.API.CommLayerUbuntu.h Kinova.API.UsbCommandLayerUbuntu.h \
		  KinovaTypes.h Robot.h

AR = ar
AROPT = rcs
LIBRARY = lib-robot.a

all: $(SOURCES) $(HEADERS) $(LIBRARY)

$(LIBRARY): $(OBJECTS)
	$(AR) $(AROPT) $@ $^

clean:
	rm $(OBJECTS) $(LIBRARY)