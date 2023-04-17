using System;
using System.Collections.Generic;
using System.Linq;
using Mars.Components.Environments;
using Mars.Components.Layers;
using Mars.Core.Data;
using Mars.Interfaces.Annotations;
using Mars.Interfaces.Data;
using Mars.Interfaces.Environments;
using Mars.Interfaces.Layers;
using NetTopologySuite.Geometries;

namespace DroneSimulation.Model;
/// <summary>
/// Initializes the Map with the given area and registers the agents.
/// 
/// For further information have a look at the MARS documentation.
/// </summary>
public class MapLayer : AbstractLayer 
{
    
    #region Properties and Fields

    [PropertyDescription(Name = "Perimeter")]
    public Perimeter Area{ get; set; }
    public List<Drone> Drone { get; set; }


    public GeoHashEnvironment<Drone> Environment { get; set; }

    #endregion
    
    /// <summary>
    /// This registers all drone agents. This needs to call the tick method of the agents.
    /// Furthermore the expansion of the simulation area is calculated using
    /// the raster layers described in config.json. An environment is created with this bounding box.
    /// </summary>
    /// <param name="layerInitData"></param>
    /// <param name="registerAgentHandle"></param>
    /// <param name="unregisterAgentHandle"></param>
    /// <returns>true if the agents where registered</returns>
    public override bool InitLayer(LayerInitData layerInitData, RegisterAgent registerAgentHandle,
        UnregisterAgent unregisterAgentHandle)
    {
        base.InitLayer(layerInitData, registerAgentHandle, unregisterAgentHandle);

        // Calculate and expand extent
        var baseExtent = new Envelope(Area.Extent.ToEnvelope());
        Console.WriteLine(new BoundingBox(baseExtent));

        // Create GeoHashEnvironment with the calculated extent
        Environment = GeoHashEnvironment<Drone>.BuildByBBox(new BoundingBox(baseExtent), 1000);

        var agentManager = layerInitData.Container.Resolve<IAgentManager>();
        Drone = agentManager.Spawn<Drone, MapLayer>().ToList();

        return Drone.Count > 0;
    }
}