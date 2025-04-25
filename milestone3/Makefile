PROJECT_NAME = lunar
EXE_NAME = sim_lunar
CC = gcc
CPP = g++

SIMULATOR_CCFLAGS = -Wall -Wpedantic -Werror -g -O0 -c -o
SIMULATOR_LDFLAGS = -no-pie -g -O0 -o 
SIMULATOR_LDLIBRARY = -L. -l$(PROJECT_NAME) -lpthread -lm

SIMULATOR_OBJECTS = configuration.o \
				simulation_parameters.o \
				main.o

all: $(EXE_NAME)

$(EXE_NAME): $(SIMULATOR_OBJECTS)
	$(CPP) $(SIMULATOR_LDFLAGS) $(EXE_NAME) $(SIMULATOR_OBJECTS) $(SIMULATOR_LDLIBRARY)

$(SIMULATOR_OBJECTS): %.o: %.c
	$(CC) $(SIMULATOR_CCFLAGS) $@ $<

clean:
	rm -f *.o
	rm -f $(EXE_NAME)

