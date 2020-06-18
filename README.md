# Bidirectional Fast Downward: Satisficing Bidirectional Heuristic Search Planner

Ryo Kuroiwa and Alex Fukunaga. Front to Front Heuristic Search for Satisficing Classical Planning. IJCAI 2020.

## Build

```
$ docker build . -t bfd
$ docker run 
```

## Planners
Attach to the container and execute following commands.
Replace `domain.pddl` and `problem.pddl` with domain and problem files that you want to solve, or specify a single SAS+ file.

### TTBS
```
$ python3 fast-downward.py domain.pddl problem.pddl --search 'bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=goal)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=initial)), reeval=NOT_SIMILAR, d_node_type=TTBS)'
```

### DNR(PP84) with n=75
```
$ python3 fast-downward.py domain.pddl problem.pddl --search bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=goal)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=initial)), reeval=NOT_SIMILAR, d_node_type=MAX_G, max_steps=75) ```
```

### DNR(OT)
```
$ python3 fast-downward.py domain.pddl problem.pddl --search bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=goal)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=initial)), reeval=ALL, d_node_type=TTBS) ```
```

### DNR(re)
```
$ python3 fast-downward.py domain.pddl problem.pddl --search bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=goal)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, fall_back_to=initial)), reeval=NOT_SIMILAR, d_node_type=MAX_G) ```
```

### SFBS
```
$ python3 fast-downward.py domain.pddl problem.pddl --search 'eager_sfbs(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true)))'
```

### biFD
```
$ python3 fast-downward.py domain.pddl problem.pddl --search 'bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_goal=true)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_initial=true)))
```

### BDD
```
$ python3 fast-downward.py domain.pddl problem.pddl --search 'bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_goal=true)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_initial=true)), bdd=true)
```

### BGG
```
$ python3 fast-downward.py domain.pddl problem.pddl --search 'bidirectional_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_goal=true)), front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_initial=true))), bgg_eval=front_to_front_hmax(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_initial=true), use_bgg=true)
```

### FDr
```
$  python3 fast-downward.py domain.pddl problem.pddl --search 'regression_eager_greedy(front_to_front_single(front_to_front_ff(transform=adapt_costs(cost_type=ONE, transform=partial_state()), partial_state=true, cache_initial=true)))'
```

### FD
```
$  python3 fast-downward.py domain.pddl problem.pddl --search'eager_greedy([ff(transform=adapt_costs(ONE))])
```
