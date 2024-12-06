#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

int P;
clock_t t;
time_t t1;

int subtractor(int A, int B) {
    int C;

    __asm__ (
        "sub %2, %1\n\t"
        "mov %1, %0"
        : "=r" (C)
        : "r" (A), "r" (B)
    );
}

int unlocker(int N, int S, int E) {
    int P = 0;
    int S_new = 0;
    int E_new = 0;
    int i;

    for (i = (N-1); i >= 0; i--) {

        int S_new = floor(S / pow(10, i));
        int E_new = floor(E / pow(10, i));
        S_new = S_new % 10;
        E_new = E_new % 10;

        if (E_new >= S_new) {
            if ((subtractor(E_new, S_new) >= 0) && (subtractor(E_new, S_new) <= 5)) {
                P += subtractor(E_new, S_new);
            }
            else {
                P += subtractor(10, E_new) + S_new;
            }
        }
        else {
            if ((subtractor(S_new, E_new) >= 0) && (subtractor(S_new, E_new) <= 5)) {
                P += subtractor(S_new, E_new);
            }
            else {
                P += subtractor(10, S_new) + E_new;
            }    
        }
    }

    return P;

}

int main(void) {
    srand((unsigned) time(&t1));

    // int N = 4;
    // int S = 1234;
    // int E = 9899;

    // int N = 0;
    // int S = 0;
    // int E = 0;

    int j = 0;

    t = clock();
    for (j = 0; j < 100000; j++) {
        //For same lengths (Comment for different lengths and uncomment this section)
        int N = 4;
        int S = rand()%9000 + 1000;
        int E = rand()%9000 + 1000;

        //For different lengths (Comment for same lengths and uncomment this section)
        // int N = rand()%100 + 1;
        // int pow1 = pow(10, N) - pow(10, N-1);
        // int S = rand()%pow1 + pow(10, N-1);
        // int E = rand()%pow1 + pow(10, N-1);

        P = unlocker(N, S, E);
    }
    t = clock() - t;
    double time_taken = ((double)(t)) / CLOCKS_PER_SEC;

    printf("%d\n", P);
    printf("%f\n", time_taken);
    // printf("%d\n", N);
    // printf("%d\n", S);
    // printf("%d\n", E);

    return 0;
}