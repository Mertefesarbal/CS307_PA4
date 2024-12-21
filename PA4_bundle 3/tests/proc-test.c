#include "../vm.c"

int main(int argc, char **argv) {
    initOS();
    fprintf(stdout, "Occupied memory after OS load:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);
    createProc("programs/simple_code.obj", "programs/simple_heap.obj");
    fprintf(stdout, "Occupied memory after program load:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);    
    loadProc(0);
    freeMem(8, reg[PTBR]);
    freeMem(9, reg[PTBR]);
    fprintf(stdout, "Occupied memory after freeing heap:\n");
    fprintf_mem_nonzero(stdout, mem, UINT16_MAX);
    return 0;
}