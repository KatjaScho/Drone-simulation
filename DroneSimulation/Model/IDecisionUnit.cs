using System.Collections.Generic;
using Mars.Components.Layers;
using Mars.Interfaces.Environments;

namespace DroneSimulation.Model;

/// <summary>
/// This interface should be used to implement a decision unit for the drone agent.
/// The implementation should decide the next state and provide the relevant information,
/// which the agent needs for this state within the getters.
/// </summary>
public interface IDecisionUnit
{
    /// <summary>
    /// This method should calculate depending on the given information the new state of the drone
    ///
    /// </summary>
    /// <param name="signalLayer"></param> A Layer which contains all relevant information about the signals
    /// <param name="perimeter"></param> The perimeter the drone moves on
    /// <param name="position"></param> THe current position of the drone 
    /// <param name="allDronePositions"></param> A dictionary which contains all drones and there positions
    /// <returns> THe next state of the drone</returns>
    public DroneStates Decide(SignalLayer signalLayer, Perimeter perimeter, 
        Position position,Dictionary<string,Drone> allDronePositions);
    
    double GetBearing();
    
    double GetDistance();
    
    VectorFeature GetNewTarget();
}