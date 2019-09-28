# Motion Planning - Bug Algorithms and Kinematics
*For optimal viewing of this document (and all `*.md` files), try opening it in a text editor that supports syntax highlighting for markdown `*.md` files (e.g. Sublime Text 2+).*

Implementation of Bug Algorithms and Basic Forwards / Inverse Kinematics Equations.

[Nicholas Rennninger](https://github.com/nicholasRenninger)

---

## How to Run the Code

This repository uses python 3, which is managed with an anaconda environment.

### Dependencies
In order to obtain all dependencies, you simply need to install anaconda for your OS, and then run from any shell with the `conda` command on its path:

`$(REPO_DIR_LOCATION) $ conda env create -f conda/environment.yml`


### Running the Source Code

To run the code:

* Activate the `conda` environment you just created in the "Dependencies" section: `$(REPO_DIR_LOCATION) $ conda activate motionPlanning1`

* Run the main python module to run both the bug algorithms and kinematics codes: `$(REPO_DIR_LOCATION) $ python main.py`


### Configuring the Code

**Bug Algorithm Configuration**
To change the scenarios for the bug algorithm code, simply modify one of the `$(REPO_DIR_LOCATION)/config/scenarioX.yaml` files to create a new set of obstacles, or to change the location of the start points.

---

## Results

<!-- ![Alt](Figures/LDBA.PNG "Specification Buchi Automaton") -->
