#include "../vm.c"

int main(int argc, char **argv) {
    initOS();
    createProc("programs/simple_code.obj", "programs/simple_heap.obj");
    loadProc(0);
    mw(0x4000, 42); // changing the value of an address to 42.
    fprintf(stdout, "%d\n",  mr(0x4000)); // reading it back.
    return 0;
}