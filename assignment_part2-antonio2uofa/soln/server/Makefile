CC = g++
LINKER = server
CC_ARGS = -Wall -g -std=c++11
CC_OBJS = server.o dijkstra.o digraph.o
PIPES = inpipe outpipe

$(LINKER): $(CC_OBJS)
	$(CC) $(CC_OBJS) -o $(LINKER)

server.o: server.cpp
	$(CC) $(CC_ARGS) -c server.cpp

dijkstra.o: dijkstra.cpp
	$(CC) $(CC_ARGS) -c dijkstra.cpp

digraph.o: digraph.cpp
	$(CC) $(CC_ARGS) -c digraph.cpp

clean:
	rm -f $(C_OBJ) $(LINKER) $(PIPES) $(CC_OBJS) *.out
	rm -f '../inpipe' '../outpipe'
