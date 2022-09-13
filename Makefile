all:path-step1 path-step2 path-step3 path-step4
path-step1: step1.cpp
	g++ -o path-step1 -std=c++11 -Wall -Werror -ggdb3 step1.cpp
path-step2: step2.cpp
	g++ -o path-step2 -std=c++11 -Wall -Werror -ggdb3 step2.cpp
path-step3: step3.cpp
	g++ -o path-step3 -std=c++11 -Wall -Werror -ggdb3 step3.cpp
path-step4: step4
	g++ -o path-step4 -std=c++11 -Wall -Werror -ggdb3 step4.cpp
clean:
	rm -f path-step1 path-step2 path-step3 path-step4 *~