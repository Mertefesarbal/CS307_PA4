#include "../vm.c"

int main(int argc, char **argv) {
    initOS();
    fprintf(stdout, "Occupied memory after OS load:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);
    return 0;
}