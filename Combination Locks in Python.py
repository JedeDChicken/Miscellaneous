from time import perf_counter
import random

def unlocker(N, S, E):
    global P
    P = 0
    S_new = 0
    E_new = 0

    for i in reversed(range(N)):
        S_new = (int(S) // (10 ** i)) % 10
        E_new = (int(E) // (10 ** i)) % 10

        if (E_new >= S_new):
            if (((E_new - S_new) >= 0) and ((E_new - S_new) <= 5)):
                P += E_new - S_new
            else:
                P += (10 - E_new) + S_new
        else:
            if (((S_new - E_new) >= 0) and ((S_new - E_new) <= 5)):
                P += S_new - E_new
            else:
                P += (10 - S_new) + E_new
                    
    return P

N = 0
S = 0
E = 0

start = perf_counter()
for i in range(100000):
    #For same lengths (Comment for different lengths and uncomment this section)
    N = 4
    S = random.randint(1000, 9999)
    E = random.randint(1000, 9999)

    #For different lengths (Comment for same lengths and uncomment this section)
    # N = random.randint(1, 100)
    # S = random.randint(10 ** (N-1), (10**N) - 1)
    # E = random.randint(10 ** (N-1), (10**N) - 1)

    unlocker(N, S, E)

t = (perf_counter() - start) * 1000
print(P)
print(t)
# print(N)
# print(S)
# print(E)