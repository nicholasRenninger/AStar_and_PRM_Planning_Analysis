# Motion Planning - A* / Dijkstra and Probabilistic Roadmap (PRM) Planning
*For optimal viewing of this document (and all `*.md` files), try opening it in a text editor that supports syntax highlighting for markdown `*.md` files (e.g. Sublime Text 2+).*

Implementation of the A* and Dijkstra optimal graph search algorithms, a Fast Probabilistic Roadmap (PRM) Planner with path smoothing, and a benchmarking suite for doing parametric performance evaluation of the planners modules. 

Built with a `networkx` Graph backbone and library implementations of:
* Set-Priority Queue (based on `heapq`)
* KDTree (based on `cKDTree`)
* Union-Find (based on `newtorkx.util`)

ADTs for speed.

[Nicholas Rennninger](https://github.com/nicholasRenninger)

---
## SLOC Stats for nerds
```bash
-------------------------------------------------------------------------------
Language                     files          blank        comment           code
-------------------------------------------------------------------------------
Python                          26           1032           2192           2298
YAML                            18            206            404            443
Markdown                         1             21              0             43
-------------------------------------------------------------------------------
SUM:                            45           1259           2596           2784
-------------------------------------------------------------------------------
```
---

## How to Run the Code

This repository uses python 3, which is managed with an anaconda environment.

### Dependencies
In order to obtain all dependencies, you simply need to install anaconda for your OS, and then run from any shell with the `conda` command on its path:

`$(REPO_DIR_LOCATION) $ conda env create -f conda/environment.yml`


### Running the Source Code

To run the code:

* Activate the `conda` environment you just created in the "Dependencies" section:
`$(REPO_DIR_LOCATION) $ conda activate motionPlanning3`

* Run the main python module to run all of the simulations:
`$(REPO_DIR_LOCATION) $ python main.py`


## Configuring the Code

### A* and Dijkstra Optimal Search Algorithms
To change the scenarios for the search algorithm simulations, simply modify one of the `$(REPO_DIR_LOCATION)/config/graphSearch_XXX.yaml` files to change the test graph or to change the location of the start / goal nodes.

### PRM planner for a point Robot
To change the scenarios for the PRM planning simulation, simply modify one of the `$(REPO_DIR_LOCATION)/config/prmPointRobot_XXX.yaml` files to change the robot or obstacle vertices, or to change the location of the start / goal state or the robot, or to change properties of the PRM planner.

### PRM planner Benchmarking
To change the scenarios for the PRM planner benchmarking, simply modify one of the `$(REPO_DIR_LOCATION)/config/prmPointRobotBenchmark_XXX.yaml` files to change the robot or obstacle vertices, or to change the location of the start / goal state or the robot, or to change properties of the PRM planner or to change the parameters you would like to vary.

---

## Results
