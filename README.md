# Drone-simulation

This project contains the simulation environment for my mainproject.
This project is used to simulate a drone swarm which locates radio signals. 

The idea behind this simulation environment is that a swarm of drones, equipped with radio receivers, can locate radio signals using the TDoA method. The TDoA (Time Difference of Arrival)-method is an algorithm where the relative time difference in receiving a signal is measured at several positions and the position of the signal is calculated based on these time differences.
The advantage of using drones is that they can position themselves quickly and optimally, even in rough terrain. In addition, they can quickly cover large reception areas. Such applications are used for the search and rescue of people, especially in rough terrain. [[1]](https://ieeexplore.ieee.org/abstract/document/8746312) [[2]](https://dl.acm.org/doi/abs/10.3233/978-1-61499-672-9-1777)

In order to use drones for such an application, a distributed control system for the drones is required. This simulation environment was created for the development and testing of such a control system. The drone agent developed here represents the digital twin of a physical drone to be used later for control. The simulation is a simplified process in which it is assumed that a approximate position is already known and the drones only have to arrange themselves around this position.

The environment is implemented based on the [MARS Framework](https://www.mars-group.org/) which is a distributed simulation framework for multi agent models. It is developed in C#.
The application consists of a geo referenced raster layer on which the agents move, a drone agent and a simple control unit.

## Model structure

The project consists of a C# application that runs the simulations and several Jupyter notebook files that are needed for configuration and visualisation of the results.

The model for the simulation environment consists of the following elements:
- `Drone`: The agent type in the model. It has various functionalities such as moving in a given direction or locating and is controlled via an IDecisionUnit. The used DecisionUnit is initialized in the init method of the agent. The drones are parameterized with the start_positions.csv file in the _Resources_ folder. To configure the heights control there a two variables. The  _MaxFlightHeight_ configures the height that the drone can fly over and the _ControlHeight_ boolean to configure whether the heights should be checkt or not. 
- `DroneStates`: This enum holds the different states the drone has. Every state describes a different functionality of the drone.
- `IDecisionUnit`: This Interface describes the control of the drone. The decision unit gets the positions of every drone, the signal positions and the area they move on for every tick to decide the next action of the drone based on the drone states.
- `MapLayer`: Registers the agent to the given area in the _Resource_ folder
- `Perimeter` Contains the .asc file from the _Resource_ folder with the area the drones are allowed to move on.
- `ElevationLayer` Contains the .asc file from the _Resource_ folder with the elevation information.
- `SignalLayer`: A vector layer with the signal information from the `test_signals.geojson` file in the _Resource_ folder
- `SimpleDecisionUnit`: A simple implementation of the `IDecisionUnit`. This is currently used to control the agent. This decision unit tries based on basic mathematic operations to move the agent to the same signal as the other agents and then position around the signal. This decision unit has no collision avoidance mechanism and no smart way to act when he leaves the allowed area or runaround obstacles.
- `Program.cs`: Creates the entire model and starts the simulation

## How to use this project
### Setup and configuration options

### Step 1: Setup
- First of all you need an working environment to run MARS simulations. Therefor you need to install the .NET-SDK on your computer
- If you want to make changes in the simulation model you also need an Integrated Development Environment (IDE). The easiest way is to install JetBrains Rider
- For more information about the overall setup you can have a look [here](https://www.mars-group.org/docs/tutorial/installation)
- Furthermore, you need docker on your machine if you want to run the visualisation
- Than you can checkout this repository and open it with rider

### Step 2: Using a different map

#### Preparation of files
- Follow the instructions on the [MARS documentation](https://mars.haw-hamburg.de/articles/core/tutorials/create_vector_layer_raster_layer.html) to get the geo referenced raster data
  - If there should be obstacles there are two Options
    - Option 1: They can be add during the creation of the grid layer
      - For this you need to create a second vector layer with the areas that should be excluded from the allowed area
      - Than select > Processing (Verarbeitung) > Toolbox (Werkzeugkiste) > Vectorlayeroverlapping (Vektorlayerüberlagerung) > Difference (Differnez) 
      - The input layer is the layer with the allowed area and the overlapping layer is the layer with the obstacles
      - Let the difference be saved to a temporarylayer and use this layer to continue with the tutorial above
    - Option 2: A Grid Layer with elevation data can be used  
      - This file can be created with QGis, a tutorial can be found [here](https://www.gis-lernen.de/gk-l4-mit-hoehendaten-arbeiten/)
    - If you dont want to use elevation data ControlHeight variable in the drone model need to be set to false   
  - This process will create a .geojson file and a .asc file, both will be needed
- To create new start positions for the agents and signals, new configuration files for both are needed
- For this you need a jupyter notebook
  - To get one the `notebookdocker.bat` or the `notebookdocker.sh` script can be used. This starts a docker container with a jupyter notebook, which can be reached under `localhost:8888`
- In the notebook the `generate_start_positions.ipynb` can be used to create a new csv configuration file for the agents
  - For this, the path must first be adapted so that it points to the newly created .geojson file. Then, depending on the maximum used number of agents, the number_of_start_positions must be adjusted.
- The `prepare_signals.ipynb`  will create new random signal positions and saves them to a .geojson file 
  - Here, you also need to change the path and the number of signals you want
  
### Step 3: Configuration of simulation

- The configuration of the simulation is located in the `config.json` file 
- If you created a new map you need to change the path for the different layers according to the location of your files
  - The perimeter needs the .asc file you created at the beginning of step 2
  - The elevation layer needs a .asc file with elevation data. This must also refer to some asc file even if no height control is used In this case the data in the file does not matter
  - The signal layer needs the generated .geojson file
- Also you need to change the configuration file of the agent to the created configuration
- If you want a different number of agents in the simulation you have to change `count`
  - NOTE: This number needs to be equal to or smaller then the number_of_start_positions when creating the .csv file

### Execution and result
### Step 4: Run simulation

#### With Rider
- To run the simulation open the DroneSimulation.sln file with rider and run it
- This will create a movement file in the `bin/Debug/net6.0` folder which can be visualized

#### Without Rider
- First use the ´run´ skript in the _DroneSimulation_ folder. This will build the project and run a simulation, but it doesnt creates the trips output file which is need for the visualisation
- But now there is a _\bin\Debug\net6.0_ folder with a ´DroneSimulation.exe´ 
  - Run the DroneSimulation.exe because this will run a simulation and create the drone_trips output file

### Step 5: Analysis

- There is a third .ipynb file, the analyze.ipynb 
- This can be run to create
  - a plot with all movements overlapping with transparency
  - a visualisation based on [Kepler gl](https://kepler.gl/)
    - NOTE: If you created a new area the path in the visualisation needs to be adjusted 




