CXX = g++
CPPFLAGS =
CXXFLAGS = -std=c++11
SOURCES = src/Robot.cpp src/Robot-API.cpp src/Robot-control.cpp \
	src/Robot-torque-control.cpp src/Robot-measure.cpp src/Robot-move.cpp\
	src/Robot-phy-parameters.cpp src/Robot-time.cpp src/Robot-utils.cpp src/Robot-try.cpp
OBJECTS = $(SOURCES:.cpp=.o)
HEADERS = Kinova.API.CommLayerUbuntu.h Kinova.API.UsbCommandLayerUbuntu.h \
	  Kinova.API.EthCommLayerUbuntu.h Kinova.API.EthCommandLayerUbuntu.h KinovaTypes.h src/Robot.h

AR = ar
AROPT = rcs
LIBRARY = librobot.a

all: $(SOURCES) $(HEADERS) $(LIBRARY)

$(LIBRARY): $(OBJECTS)
	$(AR) $(AROPT) $@ $^

clean:
	rm $(OBJECTS) $(LIBRARY)