# 2048-puzzle-AI

Implemented an iterative deepening Minmax algorithm with apha-beta pruning. The Player AI is allowed 0.2 seconds to come up with each move.

Five Heuristic functions with variable weights were used to evaluate moves:

1. Maximum tile value
2. Number of empty tiles
3. Smoothness
4. Monotonocity
5. Manhattan distance of the biggest tile from the upper-left corner


## Usage

```
python3 GameManager.py
```

## Performance
The AI achieved the 2048 tile in all the 10 test runs and even the 4096 tile in a few runs.
