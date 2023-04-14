using System;
using System.Collections.Generic;
using System.Configuration;
using System.Linq;
using Mars.Common;
using Mars.Components.Layers;
using Mars.Interfaces.Agents;
using Mars.Interfaces.Annotations;
using Mars.Interfaces.Data;
using Mars.Interfaces.Environments;
using Mars.Numerics;
using NetTopologySuite.Features;
using NetTopologySuite.Geometries;
using ServiceStack;
using Position = Mars.Interfaces.Environments.Position;

namespace DroneSimulation.Model;

/// <summary>
///     This class provides the basic implementation of the agent. This agent represents the digital twin of the drones.
///     It has all the relevant information and functionalities needed to control the drones.
///     This includes knowing the positions and targets of the other drones.
///     An instance of an IDecisionUnit is required for decision-making.
/// </summary>
public class Drone : IAgent<MapLayer>, IPositionable
{
    #region Properties and Fields
    
    /// <summary>
    /// This dictionary is used to share the positions of all agents among themselves
    /// </summary>
    private static readonly Dictionary<string,Drone> ListDronePositions = new ();

    /// <summary>
    ///     The layer which holds the signal information
    /// </summary>
    [PropertyDescription(Name = "SignalLayer")]
    public SignalLayer SignalLayer { get; set; }

    /// <summary>
    ///     The perimeter of the simulation environment
    /// </summary>
    [PropertyDescription(Name = "Perimeter")]
    public Perimeter Perimeter { get; set; }
    
    /// <summary>
    ///     The name of this agent
    /// </summary>
    [PropertyDescription(Name = "DroneName")]
    public string DroneName { get; set; }

    /// <summary>
    ///     The latitude of the current geo-referenced position of the agent
    /// </summary>
    [PropertyDescription(Name = "Latitude")]
    public double Latitude { get; set; }

    /// <summary>
    ///     The longitude of the current geo-referenced position of the agent
    /// </summary>
    [PropertyDescription(Name = "Longitude")]
    public double Longitude { get; set; }

    /// <summary>
    ///     The current position of the agent
    /// </summary>
    public Position Position { get; set; }

    /// <summary>
    ///     The unique identifier of the agent
    /// </summary>
    public Guid ID { get; set; }

    /// <summary>
    ///     The layer on which these agents live
    /// </summary>
    public MapLayer Layer { get; set; }

    private IDecisionUnit DecisionUnit { get; set; }

    private VectorFeature _currentSignal;

    #endregion

    #region Initialization

    public void Init(MapLayer layer)
    {
       // Check if the starting point is inside the perimeter. THe position is specified in the input csv file.
       if (!Perimeter.IsPointInside(new Position(Longitude, Latitude))) 
       {
            throw new Exception("Start point is not inside perimeter.");
       }
        
       ListDronePositions.Add(DroneName,this);
        
       Position = Position.CreateGeoPosition(Longitude, Latitude);
       DecisionUnit = new SimpleDecisionUnit(DroneName);
    }

    #endregion

    #region Tick
    
    public void Tick()
    {
        var state= DecisionUnit.Decide(SignalLayer, Perimeter, 
            Position, ListDronePositions);
        
        switch (state) 
        {
          case DroneStates.Locating:
              Locating();
              break;
          case DroneStates.Wait:
              Waiting();
              break;
          case DroneStates.MoveTowards:
              MoveTowards(DecisionUnit.GetBearing(), DecisionUnit.GetDistance());
              break;
          case DroneStates.SetNewTarget:
              _currentSignal = DecisionUnit.GetNewTarget();
              break;
          default:
              Waiting();
              break;
        }
    }

    private void MoveTowards(double bearing, double distance)
    {
        Position = Layer.Environment.MoveTowards(this, bearing, distance);
    }
   
    private void Locating()
    {
        foreach (VectorFeature signal in SignalLayer)
        {
            if (signal.Equals(_currentSignal))
            {
                
                var structuredAttributes = signal.VectorStructured.Attributes;
                structuredAttributes["located"] = "true";
                signal.VectorStructured.Attributes = structuredAttributes;
                break;
            }
        }
        
    }

    private double _waitingBearing = 0;
    private void Waiting()
    {
        //For some reason kepler only shows the agents when they move, as soon as the position stays the same they disappear
        //from the map. To deal with this the agents move minimally in place when they are waiting. 
        _waitingBearing = (_waitingBearing + 90) % 360;
        Position = Layer.Environment.MoveTowards(this, _waitingBearing, 0.1);
    }

    public VectorFeature GetSignal()
    {
        return _currentSignal;
    }

    #endregion
}