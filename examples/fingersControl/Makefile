all: fingerControl

fingerControl: main.o
	 g++ -o fingerControl main.o -L/../../lib-robot.a

main.o: main.cpp
	 g++ -std=c++11 -c main.cpp
     
clean:
	 rm main.o fingerControl