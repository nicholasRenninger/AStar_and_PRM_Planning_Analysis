# Motion Planning - Configuration Space Generation and Planning with Gradient Descent / WaveFront 
*For optimal viewing of this document (and all `*.md` files), try opening it in a text editor that supports syntax highlighting for markdown `*.md` files (e.g. Sublime Text 2+).*

Implementation of a 2-D 2DoF manipulator configuration space visualizer, as well as implementations of a gradient descent and wavefront planner for this manipulator.

[Nicholas Rennninger](https://github.com/nicholasRenninger)

---

## How to Run the Code

This repository uses python 3, which is managed with an anaconda environment.

### Dependencies
In order to obtain all dependencies, you simply need to install anaconda for your OS, and then run from any shell with the `conda` command on its path:

`$(REPO_DIR_LOCATION) $ conda env create -f conda/environment.yml`


### Running the Source Code

To run the code:

* Activate the `conda` environment you just created in the "Dependencies" section:
`$(REPO_DIR_LOCATION) $ conda activate motionPlanning2`

* Run the main python module to run all of the simulations:
`$(REPO_DIR_LOCATION) $ python main.py`


### Configuring the Code

## C-Space visualizer for a translating, rotating robot and obstacle
To change the scenarios for the c-space visualization simulation, simply modify one of the `$(REPO_DIR_LOCATION)/config/cspace_XXX.yaml` files to change the robot or obstacle vertices, or to change the location of the start / goal state or the robot.

## Gradient descent planner for a point robot
To change the scenarios for the gradient descent planning simulation, simply modify one of the `$(REPO_DIR_LOCATION)/config/gradient_XXX.yaml` files to change the robot or obstacle vertices, or to change the location of the start / goal state or the robot, or to change the planner settings.

## Wavefront planner for a point Robot
To change the scenarios for the wavefront planning simulation, simply modify one of the `$(REPO_DIR_LOCATION)/config/wavefront_XXX.yaml` files to change the robot or obstacle vertices, or to change the location of the start / goal state or the robot, or to change properties of the wavefront planner.

## 2-DOF Manipulator C-Space visualizer, inverse kinematics, and wavefront planning
To change the scenarios for the manipulator simulation, simply modify one of the `$(REPO_DIR_LOCATION)/config/manipulator_XXX.yaml` files to change the robot or obstacle vertices, or to change the location of the start / goal state or the robot, or to change properties of the wavefront planner.

---

## Results

### Manipulator with Wavefront Planner

#### Environment 1
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env1-workspace.gif"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env1-wavefrontPlannerwavefront.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/a/manipulator_env1-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env1-workspace.png"/>

#### Environment 2
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env2-workspace.gif"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env2-wavefrontPlannerwavefront.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/a/manipulator_env2-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env2-workspace.png"/>

#### Environment 3
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env3-workspace.gif"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env3-wavefrontPlannerwavefront.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/a/manipulator_env3-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/manipulator_env3-workspace.png"/>


### Point Robot with Gradient Descent Planner

#### Environment 1
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env1-workspace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env1-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env1-gradientPlanner.png"/>

#### Environment 2
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env2-workspace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env2-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env2-gradientPlanner.png"/>

#### Environment 3
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env3-workspace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env3-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/gradient_env3-gradientPlanner.png"/>

### Point Robot with Wavefront Planner

#### Environment 2
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/wavefront_env2-workspace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/wavefront_env2-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/wavefront_env2-wavefront.png"/>

#### Environment 3
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/wavefront_env3-workspace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/wavefront_env3-cSpace.png"/>
<img src="https://github.com/nicholasRenninger/cSpaceViz_Gradient_Wavefront_planners/blob/master/figures/wavefront_env3-wavefront.png"/>
