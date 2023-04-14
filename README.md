# Drone-simulation

This Project contains the Simulation environment for my mainproject and my Master Degree.
This Project is used to simulate a drone swarm which locates radio signals. 

The idea behind this simulation environment is that a swarm of drones equipped with radio receivers can locate radio signals using the TDoA method. The TDoA (Time Difference of Arrival)-method is a algorithm where the relative time difference in receiving a signal is measured at several positions and the position of the signal is calculated based on these time differences.
The advantage of using drones is that they can position themselves quickly and optimally even in rough terrain. In addition, they can quickly cover large reception areas. Such applications are used for the search and rescue of people, especially in rough terrain. [[1]](https://ieeexplore.ieee.org/abstract/document/8746312) [[2]](https://dl.acm.org/doi/abs/10.3233/978-1-61499-672-9-1777)

In order to use drones for such an application, a distributed control system for the drones is required. This simulation environment was created for the development and testing of such a control system. The drone agent developed here represents the digital twin of the physical drones, so they can be used later for to control. The simulation is a simplified process in which it is assumed that a rough position is already known and the drones only have to arrange themselves around this position.

The environment is implemented based on the [MARS Framework](https://www.mars-group.org/) which is a distributed simulation framework for multi agent models. It is developed in C#.
The application consists of a geo referenced raster layer on which the agents move, a drone agent and a simple control unit.

## Model Structure

The project consists of a C# application that runs the simulations and several Jupyter notebook files that are needed for configuration and visualisation of the results.

The model of the simulation environment consists of the following elements:
- `Drone`: The agent type in the model. It has various functionalities such as moving in a given direction or locating and is controlled via an IDecisionUnit. THe used DecisionUnit is initialized in the init Method of the Agent. The drones are parameterized with the drone.csv file in the _Resources_ folder 
- `DroneStates`: This enum holds the different states the drone has. Every state describes a different functionality of the drone.
- `IDecisionUnit`: This Interface describes the control of the drone. The decision unit gets the positions of every drone, the signal positions and the area they move on for every tick to decide the next state of the drone.
- `MapLayer`: Registers the Agent to the given area in the _Resource_ folder
- `Perimeter` Contains the .asc file from the _Resource_ folder wth the area the drones are allowed to move on. 
- `SignalLayer`: A vector layer with the signal information from the `test_signals.geojson` file in the Resource folder
- `SimpleDecisionUnit`: A simple implementation of the `IDecisionUnit`. This is currently used to control the agent. THis decision unit tries based on basic mathematik operations to move the Agent to the same Signal as the other agents and then position arount the signal. This decision unit has no collision avoidance mechanism and no smart way to act when he leaves the allowed area.
- `Program.cs`: Creates the entire model and starts the simulation

## How to use this project
### Setup and Configuration options

### Step 1: Setup
- First of all you need an working environment to run MARS simulations. Therefor you need to install the .NET-SDK on your computer and an Integrated Development Environment (IDE). The easiest way is to install JetBrains Rider. For more information about the overall setup you can have a look [here](https://www.mars-group.org/docs/tutorial/installation)
- Furthermore you need docker on your machine if you want to run the visualisation.
- Than you can checkout this repository and open it with rider

### Step 2: Using a different map

#### Preparation of files
- Follow the instruction on the [MARS documentation](https://mars.haw-hamburg.de/articles/core/tutorials/create_vector_layer_raster_layer.html) to get the geo referenced raster data
  - This process will create a .geojson file and a .asc file both will be needed
- To create new start positions of the agents and signals in your area we need new configuration files for both
- For this you need a jupyter notebook
  - To get one the `notebookdocker.bat` or the `notebookdocker.sh` script can be used. This starts a docker container with a jupyter notebook, which can be reached under `localhost:8888`
- In the notebook the `generate_start_positions.ipynb` can be used to create a new csv configuration file for the agents
  - For this, the path must be adapted so that it points to the newly created geojson file. Secondly, depending on the maximum number of agents to be used, the number_of_start_positions must be adjusted.
- The `prepare_signals.ipynb`  will create new random signal positions and safes them to a geojson file. 
  - Here, you also need to change the path and the number of signals you want
  
### Step 3: Configuration of Simulation

- The configuration of the simulation is located in the `config.json` file 
- If you created a new map you need to change the path for the different layers according to the location of your files
  - The Perimeter needs the .asc file you created at the beginning of step 2
  - THe Signal layer needs the generated .geojson file
- Also you need to change the configuration file of the agent to the created configuration
- If you want a different number of agents in the simulation you have ot change `count`
  - NOTE This number needs to be equals or smaller to the number_of_start_positions when creating the csv file

### Execution and result
### Step 4: Run Simulation

- To run the simulation open the DroneSimulation.sln file with rider and run it
- This will create a movement file in the `bin/Debug/net6.0` folder which can be visualized.

### Step 5: Analysis

- There is a third .ipynb file the analyze.ipynb. 
- This can be run to create
  - a plot with all movements overlapping with transparency
  - a visualisation based on [Kepler gl](https://kepler.gl/)
    - NOTE If you created a new area the Path in the visualisation needs to be adjusted 




