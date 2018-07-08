all: fingerControl

fingerControl: main.o
	 g++ -o torqueControl main.o -v ../../librobot.a

main.o: main.cpp
	 g++ -std=c++11 -c -v -L ../../librobot.a main.cpp
     
clean:
	 rm main.o torqueControl