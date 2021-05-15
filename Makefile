CC = gcc
RM = rm -vf

EDCFLAGS := -O2 -Wall -I include/ $(CFLAGS) $(DEBUG)
EDLDFLAGS := -lpthread -lm $(LDFLAGS)

SRC_DIR := src
SRC := $(wildcard $(SRC_DIR)/*.c)
COBJ = $(SRC:$(SRC_DIR)/%.c=$(SRC_DIR)/%.o)

all: ground space

ground: src/ground.c
	$(CC) $(EDCFLAGS) src/ground.c -o ground.out $(EDLDFLAGS)

space: src/space.c
	$(CC) $(EDCFLAGS) src/space.c -o space.out $(EDLDFLAGS)

.PHONY: clean

clean:
	$(RM) *.out



