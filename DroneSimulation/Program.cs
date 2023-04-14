using System;
using System.Collections.Generic;
using System.IO;
using DroneSimulation.Model;
using Mars.Components.Starter;
using Mars.Interfaces.Environments;
using Mars.Interfaces.Model;

namespace DroneSimulation;

internal static class Program
{
    public static void Main(string[] args)
    {
       
        // This scenario consists of:
        // 1. a model (represented by the model description)
        // 2. a simulation configuration (see config.json)
            
        // Create a new model description that holds all parts of the model (agents, entities, layers)
        var description = new ModelDescription();
        description.AddLayer<MapLayer>();
        description.AddLayer<Perimeter>();
        description.AddLayer<SignalLayer>();

        description.AddAgent<Drone, MapLayer>();
            
        // Scenario definition: Use config.json that holds the specification of the scenario
        var file = File.ReadAllText("config.json");
        var config = SimulationConfig.Deserialize(file);
            
        // Create simulation task
        var simStarter = SimulationStarter.Start(description, config);
            
        // Run simulation
        var results = simStarter.Run();
            
        // Feedback to user that simulation run was successful
        Console.WriteLine($"Simulation execution finished after {results.Iterations} steps");
    }
}