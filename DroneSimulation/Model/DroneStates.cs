namespace DroneSimulation.Model;

/// <summary>
/// This enum  contains all possible states that the drone can assume
/// </summary>
public enum DroneStates
{
    SetNewTarget,
    MoveTowards,
    Wait,
    Locating,
    SignalUnreachable
}