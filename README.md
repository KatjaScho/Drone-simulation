# Drone-simulation

This Projct contains the Simulation enviroment for my Mainprojekt and my Master Degree. The enviroment is implementet based on the MARS Framework and is used to simulate a drone swarm which can locate radio signals.

Was ist mars?

Was ist TDoA und wie funktionierts?
Was macht diese simulationsumgebung?
Anwendungsbeispiel: Ortung von lawienen opfern

## How to use this projekt
### Setup

First of all you need an working enviroment to run MARS simulations. Therefor you need to install the .NET-SDK on your computer and an Integrated Development Environment (IDE). The easiest way is to install JetBrains Rider. For more information about the overall setup you can have a look here(https://www.mars-group.org/docs/tutorial/installation) Furthermore you need docker on your maschine if you want to run the visualisation.

Than you can checkout this repository.

To see the visualisation of the last simulation you can either open the kepler_demo.html file or you need to run the notebookdocker.bat file. This will start a docker container with an running jupiter notebook this is available under localhost:8888. There you need to run the analyse.ipyth file. This will ploth a headmap of the movements of all agents and start a kepler visuaisation, within the notebook and safes it as an html file in the last cell. There are also two more notebooks available. One to create a new startig csv file for the agents and one to generate new random signals within the given area.


### Configuration options

### Runing a Simulation

## Model Structure

