all: fingerControl

fingerControl: main.o
	 g++ -o fingerControl main.o ../../librobot.a -ldl

main.o: main.cpp
	 g++ -std=c++11 -c -L ../../librobot.a main.cpp
     
clean:
	 rm main.o fingerControl
