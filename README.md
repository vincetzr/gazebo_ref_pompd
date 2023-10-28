# Reference POMDP Solver

Implementation of the Reference POMDP Solver

This code is based on [pomdp_py](https://github.com/h2r/pomdp-py). As the new solver is written in
Python, we de-Cythonised the implementation of POMCP for a fair comparison.

Dependencies
- numpy
- scipy
- tqdm
- matplotlib

Algorithm implementations
- RefSolver `pomdp_py/algorithms/ref_solver_clean.py`

Problem implementations

- Gridworld `problems/gridworld/*`

How to run

```bash
# Linux
PYTHONPATH="$PWD" python problems/gridworld/gridworld_dzs_60x60.py
# Windows
set PYTHONPATH=%cd%
python problems/gridworld/gridworld_dzs_60x60.py
```

## Note:
Tested on Python 3.10