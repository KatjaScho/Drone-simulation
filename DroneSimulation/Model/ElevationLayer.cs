using Mars.Components.Layers;
using Mars.Interfaces.Environments;

namespace DroneSimulation.Model;

/// <summary>
/// This ElevationLayer contains an .asc file with the elevation information 
/// </summary>
public class ElevationLayer : RasterLayer
{
    
    /// <summary>
    ///     Checks for a coordinate the elevation value.
    /// </summary>
    /// <param name="coordinate">The coordinate to check</param>
    /// <returns>
    ///     Returns the elevation or -1 if no information is available for this coordinate
    /// </returns>
    public double GetHeights(Position coordinate)
    {
        if (Extent.Contains(coordinate.X, coordinate.Y))
        {
            return GetValue(coordinate);
        }
        else
        {
            return -1;
        }
    }
}