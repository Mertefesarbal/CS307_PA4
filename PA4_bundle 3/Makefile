C = gcc
CFLAGS = -std=c11 -Wall

MAIN = main.c
VM = vm

PROGRAM1 = programs/simple
PROGRAM2 = programs/brk
PROGRAM3 = programs/brk2
PROGRAM4 = programs/yld

OBJ1 = programs/simple_code.obj programs/simple_heap.obj
OBJ2 = programs/brk_code.obj programs/brk_heap.obj
OBJ3 = programs/brk2_code.obj programs/brk2_heap.obj
OBJ4 = programs/yld_code.obj programs/yld_heap.obj

TEST1 = tests/initos-test
TEST2 = tests/mem-test
TEST3 = tests/mem-test2
TEST4 = tests/proc-test
TEST5 = tests/mw-mr-test
TEST6 = tests/mw-mr-test2

.PHONY: all clean programs tests sample

all: clean programs tests sample

programs: $(PROGRAM1).c $(PROGRAM2).c $(PROGRAM3).c $(PROGRAM4).c
	@$(C) $(CFLAGS) $(PROGRAM1).c -o $(PROGRAM1)
	@$(C) $(CFLAGS) $(PROGRAM2).c -o $(PROGRAM2)
	@$(C) $(CFLAGS) $(PROGRAM3).c -o $(PROGRAM3)
	@$(C) $(CFLAGS) $(PROGRAM4).c -o $(PROGRAM4)

	@$(PROGRAM1)
	@$(PROGRAM2)
	@$(PROGRAM3)
	@$(PROGRAM4)

	@rm $(PROGRAM1) $(PROGRAM2) $(PROGRAM3) $(PROGRAM4)

tests: $(TEST1).c $(TEST2).c $(TEST3).c $(TEST4).c $(TEST5).c $(TEST6).c
	@$(C) $(CFLAGS) $(TEST1).c -o $(TEST1)
	@$(C) $(CFLAGS) $(TEST2).c -o $(TEST2)
	@$(C) $(CFLAGS) $(TEST3).c -o $(TEST3)
	@$(C) $(CFLAGS) $(TEST4).c -o $(TEST4)
	@$(C) $(CFLAGS) $(TEST5).c -o $(TEST5)
	@$(C) $(CFLAGS) $(TEST6).c -o $(TEST6)

sample: $(MAIN)
	@$(C) $(CFLAGS) $(MAIN) -o $(VM)

clean:
	@rm -f $(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(TEST1) $(TEST2) $(TEST3) $(TEST4) $(TEST5) $(TEST6) $(VM)