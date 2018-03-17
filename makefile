all:
	g++ -std=c++14 -Wall -fopenmp -O3 main.cpp

benchmark:
	g++ -std=c++14 -Wall -fopenmp -O3 benchmark.cpp

benchmark2:
	g++ -std=c++14 -Wall -fopenmp -O3 benchmark2.cpp
