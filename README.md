# Sokoban solver

[![make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121/badge.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/r/Shayidinadapa/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/r/Shayidinadapa/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A fast push-optimal Sokoban solver, built for make.fun's
[Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121)
bounty ($100 in USDC).

The levels are a hallway with independent rooms off it, so the solver splits
each level into its rooms, solves every room push-optimally with A* (box
configurations + player reachability, deadlock pruning, assignment heuristic),
then joins the rooms with the player's walks through the hallway. The total
push count is the sum of the rooms' optima, which is the level's push-optimal
count.

## Run

```sh
node main.ts
```

It reads every instance under `/input/<id>/level.txt` and writes
`/output/<id>/solution.txt` (moves in `u`, `d`, `l`, `r`).

Referral code: `1bz6mt7h` — https://make.fun/r/1bz6mt7h
