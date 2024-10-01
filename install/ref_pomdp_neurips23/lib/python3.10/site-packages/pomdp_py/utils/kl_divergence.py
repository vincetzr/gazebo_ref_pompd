"""Computing Kullback-Leibler divergence"""

import math

def expand_support(d, support, epsilon=1e-9):
    c = {}
    for omega in support:
        if d.get(omega) is None:
            c[omega] = epsilon
        else:
            c[omega] = d[omega]
    
    return c    

def kl(d1, d2):
    
    if not isinstance(d1, dict) or not isinstance(d2, dict):
        raise ValueError("One of the inputs is not a dict.")
    
    if len(d1) != len(d2):
        raise ValueError("Inputs have different length.")
        
    kl = 0.0
    for omega in d1.keys():
        if d1[omega] < 0.0 or d2[omega] < 0.0:
            raise ValueError('An input distribution has negative probability.')
        if d2[omega] == 0.0 and d1[omega] > 0.0:
            raise ValueError("d1 has positive support where d2 does not.")
        if d2[omega] > 0.0 and d1[omega] > 0.0:
            kl += d1[omega] * math.log(d1[omega]/d2[omega])
        
    return kl