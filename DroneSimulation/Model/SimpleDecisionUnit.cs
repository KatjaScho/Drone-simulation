using System;
using System.Collections.Generic;
using System.Linq;
using Mars.Common;
using Mars.Components.Layers;
using Mars.Interfaces.Environments;
using Mars.Numerics;
using NetTopologySuite.Geometries;
using Position = Mars.Interfaces.Environments.Position;

namespace DroneSimulation.Model;

/// <summary>
/// This is a possible decision unit for a drone agent.
/// This decision unit makes decisions based on simple algorithms, it does not ensure that the best possible decision is made.
/// It is just a simple implementation for demonstration purpose,
/// therefore no collision avoidance or speed calculations are implemented .
/// </summary>
public class SimpleDecisionUnit : IDecisionUnit
{
    /// <summary>
    /// The distance the agent should move per tick (see config.json for time equivalent)
    /// </summary>
    private const double DistanceInMeter = 10.0;
   
    /// <summary>
    /// The distance the drone should maintain from the target in meters
    /// </summary>
    private const int DistanceToTarget = 300;
    
    /// <summary>
    /// The maximum time a drone should waits at a signal for other drones if they are the whole time alone
    /// at the signal the number is equivalent to ticks
    /// </summary>
    private const int MaximumWaitingTimeAtOneSignal = 4000;
  
    /// <summary>
    /// The duration the drone should stay at a signal after formation
    /// to simulate the needed time for localisation in ticks (see config.json for time equivalent)
    /// </summary>
    private const int DurationNeededForLocation = 1000;
    
    /// <summary>
    /// The name of the agent the decision unit instance belongs to
    /// </summary>
    private readonly string _droneName;
    
    /// <summary>
    /// The current target for this agent
    /// </summary>
    private VectorFeature _target;
    
    /// <summary>
    /// The position of the target
    /// </summary>
    private Position _targetPosition;
    
    /// <summary>
    /// The current bearing the agent should move towards
    /// </summary>
    private double _bearing;
    
    /// <summary>
    /// The calculated state
    /// </summary>
    private DroneStates _currentState;
    
    /// <summary>
    /// The following variables are used to store internal states that are needed for the decision making strategy.
    /// </summary>
    private bool _needsNewTarget = true;
    private bool _waitingForOthers;
    private bool _reachedTarget;
    private int _maximumWaitingTimeAtOneSignal;
    private int _durationNeededForLocating;
    private int _droneCountAtSameTarget;
    private Dictionary<string, Drone> _droneAtSameTargetList;
    private int _ticksSinceLeftOfArea;
    
    public SimpleDecisionUnit(string droneName)
    {
        _droneName = droneName;
    }
 
    public DroneStates Decide(SignalLayer signalLayer, Perimeter perimeter,
        Position dronePosition, Dictionary<string,Drone> allDronePositions)
    {
        // First of all, there are two main phases for the task of the drone:
        //  1. The searching and moving towards a target part of the task (mostly independent)
        //  2. When a target is reached, the waiting and positioning part of the task (collaborative)
        // The drone switches between this two phases, depending on its current state.
        // NOTE: The second phase must not be completed successfully (target is located) to switch back.
        
        // This needs to be checked first.
         if (!_reachedTarget) 
         {   
             // This part is phase 1.
             // If this is the first tick, we switched back from phase 2 and we need a new target.
            if (_needsNewTarget)
            {
                _target = FindNewTarget(allDronePositions, dronePosition,
                     perimeter, signalLayer);
                
                if (_target.IsEqual(null))
                {
                    Console.WriteLine("There seems to be no targets in this area at all, so I will stay where I am");
                }
                else
                { 
                    var targetPoint =(Point) _target.VectorStructured.Geometry; 
                    _targetPosition = new Position(targetPoint.X , targetPoint.Y);
                }
                _currentState = DroneStates.SetNewTarget;
                return _currentState;
            }
            
            // SUMMARY: Phase 1 --> We have a target to move to and we are currently on our way to this target,
            // so first we need to check if we haven't left the permitted area.
            if (!(perimeter.IsPointInside(dronePosition)))
            {
                // We left the permitted area, so change bearing to hopefully get back in it.
                if (_ticksSinceLeftOfArea<15)
                {
                    //The permitted area has not been left for long, so keep trying to avoid the obstacle 
                    _bearing = (dronePosition.GetBearing(_targetPosition)+160)%360;
                    _ticksSinceLeftOfArea++;
                }
                else if (_ticksSinceLeftOfArea < 30)
                {
                    _bearing = (dronePosition.GetBearing(_targetPosition)-160)%360;
                    _ticksSinceLeftOfArea++;
                }
                else
                {
                     // The drone appears to have drifted off course therefore on a direct return path 
                     _bearing = dronePosition.GetBearing(_targetPosition);
                }
                
                _currentState = DroneStates.MoveTowards;
                return _currentState;
            }

            _ticksSinceLeftOfArea = 0;
            
            // SUMMARY: Phase 1 --> We have a target to move to and are in the permitted area.
            // Next check: Do we reached the target already?
            if (_targetPosition.DistanceInMTo(dronePosition) < DistanceToTarget)
            {
                // The target is reached, so set all internal states to change to phase 2 in the next tick!
                _reachedTarget = true;
                _maximumWaitingTimeAtOneSignal = MaximumWaitingTimeAtOneSignal;
                _durationNeededForLocating = DurationNeededForLocation;
                _waitingForOthers = true;
                _currentState = DroneStates.Wait;
                return _currentState;
            }
            
            // SUMMARY: Phase 1 --> We have a target to move to, are in the permitted area and haven't reached the target yet,
            // so just move towards the target. 
            _bearing = dronePosition.GetBearing(_targetPosition);
            _currentState = DroneStates.MoveTowards;
            return _currentState; 
         }
         else 
         {
             // We are in Phase 2 of the task.
             // First we need to check the maximum waiting time at a signal to take care that we move somewhere else
             // if nobody is coming to this target (prevents a dead lock).
            if (_maximumWaitingTimeAtOneSignal <= 0)
            {
                // Already waited too long here and nobody seems to come, so we need to move on and try a new target.
                // Reset all internal states to indicate change to phase 1 in the next tick.
                _reachedTarget = false;
                _needsNewTarget = true;
                _waitingForOthers = false;

                _currentState = DroneStates.Wait;
                return _currentState;
            }
            else
            {
                // SUMMARY: Phase 2 --> Haven't waited too long until now.
                // Check if we are waiting for others to come to this target.
                if (_waitingForOthers)
                {
                    // Check current count at this target.
                    _droneAtSameTargetList = NumberOfDronesAtTarget(allDronePositions);
                    _droneCountAtSameTarget = _droneAtSameTargetList.Count;
                }
               
                // SUMMARY: Phase 2 --> Haven't waited too long until now and the calculated number of drones are here already.
                // If there are four drones at this target its enough for a good localisation.
                if (_droneCountAtSameTarget >= 4)
                {
                    
                    // Check if the position is already good and move if it is not.
                    // Also we now don't need to wait for others because we are already enough drones.
                    DroneStates state = CheckFormation(_droneAtSameTargetList,dronePosition);
                    _waitingForOthers = false;
                    
                    // If we need to move than move.
                    if (state.Equals(DroneStates.MoveTowards))
                    {
                            _currentState = state;
                            return _currentState;
                    }

                    // We don't need to move so check how long we are currently waiting for localization.
                    // If we waited enough time set state to locating and prepare switch to phase 1 again.
                    if (_durationNeededForLocating == 0)
                    {
                        _reachedTarget = false;
                        _needsNewTarget = true;
                        _currentState = DroneStates.Locating;
                        return _currentState;
                    }
                    
                    // We don't need to move but haven't waited long enough until now so just wait.
                    _durationNeededForLocating--;
                    
                    _currentState = state; 
                    return _currentState;
                    

                }
                // SUMMARY: Phase 2 --> Haven't waited too long until now, calculated number of drones and we are not enough.
                // If there are less then four drones also check formation to prepare a better outcome positioning
                // until the others arrive.
                else
                {
                    DroneStates state = CheckFormation(_droneAtSameTargetList, dronePosition);
                    
                    // The fewer drones, the faster they should look for a new target.
                    _maximumWaitingTimeAtOneSignal -= (4 - _droneCountAtSameTarget);
                    _currentState = state;
                    return _currentState;
                }
            } 
         }
    }

    public double GetBearing()
    {
        return _bearing;
    }

    public double GetDistance()
    {
        return DistanceInMeter;
    }

    public VectorFeature GetNewTarget()
    {
        return _target;
    }

    /// <summary>
    /// This method checks the own position depending on the position of the others and moves if the position isn't good.
    /// The following criteria are included in the check:
    ///     1. The distance to the target
    ///     2. The distance to the other drones
    ///         --> This distance depends on the number of drones at this target.
    /// </summary>
    /// <param name="droneAtSameTarget"> A list with the drones at the target which should be included in the check</param>
    /// <param name="dronePosition"> The own position</param>
    /// <returns> Whether the drone should move in the direction of the set bearing or stay where it is</returns>
    private DroneStates CheckFormation(Dictionary<string, Drone> droneAtSameTarget, Position dronePosition)
     {
         // With four drones the ideal positioning should be a square, therefore a quarter of the circleference is taken as the
         // optimal distance between the drones.
         // To ensure that the drones could also position with more than four drones the radius of the circle is increased if there are more than four drones.
         double circumference = ((DistanceToTarget +(10*(droneAtSameTarget.Count-4)))* 2.0) * Math.PI;
         double distanceBetweenDrones = circumference/ 4.0;
         
         double bearing = 0;
         double averageLat =0;
         double averageLong=0;
         int averageBearingCount = 0;
         foreach (KeyValuePair<string, Drone> droneValuePair in droneAtSameTarget)
         {
             Drone drone = droneValuePair.Value;
            
             // Only take the other drones into account.
             if (!droneValuePair.Key.Equals(_droneName)) 
             {
                 // If the distance to the other drone is too small add the opposite bearing, so we move away from this drone
                 // and add the position of the other drone. 
                 if (drone.Position.DistanceInMTo(dronePosition) < (distanceBetweenDrones*0.7))
                 {
                     averageLat += drone.Position.Latitude;
                     averageLong += drone.Position.Longitude;
                     averageBearingCount++;
                     bearing += drone.Position.GetBearing(dronePosition);
                 }
             }
             
                 
         }

         // If there where no bearings added to the average, this instance has enough distance to everyone and doesn't need to move.
         if (averageBearingCount >0)
         {
            // The own position is added and the average values for the positions and bearings are calculated.
             averageLat += dronePosition.Latitude;
             averageLong += dronePosition.Longitude; 
             averageLat /= (averageBearingCount+1);
             averageLong /= (averageBearingCount+1);
             
     
             // The average position indicates if all drones are positioned around the target.
             // If this position isn't next to the target this should be included into the bearing this instance should move.
             Position centerPosition = new Position(averageLong, averageLat);
             if (centerPosition.DistanceInMTo(_targetPosition) > 50)
             {
                 bearing += centerPosition.GetBearing(_targetPosition); 
                 bearing /= (averageBearingCount+1);
             }
             else
             {
                 bearing /= averageBearingCount;
                 
             }
             
             // If we move to far away from the target forget everything else and we will fly back to the target in a small circle.
             if(dronePosition.DistanceInMTo(_targetPosition)>DistanceToTarget+ (5*(droneAtSameTarget.Count-4)) )
             {
                 bearing = (dronePosition.GetBearing(_targetPosition)+ 80) %360;
                 
             }

             _bearing = bearing;
             return DroneStates.MoveTowards;
         }
         else
         {
             // The current position is fine so wait what the others will do.
            return DroneStates.Wait;
         }
         
     }

    /// <summary>
    /// Checks the number of drones at the target of this drone
    /// </summary>
    /// <param name="listDronePositions"> The current target and position of all drones</param>
    /// <returns> A dictionary with all drones at this the same target</returns>
    private Dictionary<string, Drone> NumberOfDronesAtTarget( Dictionary<string, Drone> listDronePositions)
    {
        Dictionary<string, Drone> dronesAtSameTarget = new Dictionary<string, Drone>();
        
        foreach (KeyValuePair<string, Drone> drone in listDronePositions)
        {
            // Check for the correct target and
            // the doubled distance to the current target so there is a tolerance range the drones are allowed to move in.
            if (drone.Value.GetSignal().IsEqual(_target) &&  drone.Value.Position.DistanceInMTo(_targetPosition) < (2*DistanceToTarget))
            {
                dronesAtSameTarget.Add(drone.Key, drone.Value);
            }
        }

        return dronesAtSameTarget;
    }

   

    /// <summary>
    /// This method searches for a new target depending on the positions of all drones,
    /// to increase the possibility that the drones fly to the same target.
    /// </summary>
    /// <param name="allDronePositions"> A dictionary with all positions</param>
    /// <param name="dronePosition"> The own position</param>
    /// <param name="perimeter"> The area the signals are in </param>
    /// <param name="signalLayer"> The signals</param>
    /// <returns> The new signal to move to or null if there is no signal</returns>
    private VectorFeature FindNewTarget(Dictionary<string, Drone> allDronePositions, Position dronePosition,
        Perimeter perimeter, SignalLayer signalLayer)
    {
        double latitudeValues = 0;
        double longitudeValues = 0;
            
        // First of all, an average position is calculated with the positions of all drones.
        foreach (var position in allDronePositions)
        {
                var valuePosition = position.Value.Position;
                latitudeValues += valuePosition.Latitude;
                longitudeValues += valuePosition.Longitude;
        }

        var averageLatitude = latitudeValues / allDronePositions.Count;
        var averageLongitude = longitudeValues / allDronePositions.Count;
        var positionToSearchFrom= new [] { averageLongitude, averageLatitude };
        
        // This position is used to explore the signal layer and get a list with all signals sorted by distance. 
        var listSignals = signalLayer.Explore(positionToSearchFrom).ToList();
        if (listSignals.Any())
        {
            // Fallback if all signals are located already.
            var nearestSignal = listSignals.First();
            
            // Search for nearest signal which is not located yet.
            foreach (var signal in listSignals)
            {
                if ((string)signal.VectorStructured.Attributes["located"] == "false")
                {
                    nearestSignal = signal; 
                    break;
                }
            }


            // Get coordinates of this signal.
            var signalLocation = (Point)nearestSignal.VectorStructured.Geometry;
            _targetPosition = new Position(signalLocation.X , signalLocation.Y);

            // Get bearing to move to the signal.
            _bearing = dronePosition.GetBearing(_targetPosition);
            
            // Now we have a new signal, we don't need to search for another one
            // until this one is used for the second phase.
            _needsNewTarget = false;
            
            return nearestSignal;
        }
        else
        {
            // If there is no signal at all
            return null;
        }
    }
}