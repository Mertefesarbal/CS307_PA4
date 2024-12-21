#include "../vm.c"

int main(int argc, char **argv) {
    initOS();
    fprintf(stdout, "Occupied memory after OS load:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);
    allocMem(4096, 0, UINT16_MAX, UINT16_MAX);
    fprintf(stdout, "Occupied memory after allocation:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);
    freeMem(0, 4096);
    fprintf(stdout, "Occupied memory after freeing:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);

    return 0;
}