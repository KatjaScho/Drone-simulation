using Mars.Components.Layers;
using Mars.Interfaces.Environments;

namespace DroneSimulation.Model;

/// <summary>
/// This RasterLayer contains an .asc file with the area the drones are allowed to move in.
/// </summary>
public class Perimeter : RasterLayer
{
    /// <summary>
    ///     Checks for a coordinate whether this point is inside the perimeter or not.
    /// </summary>
    /// <param name="coordinate">The coordinate to check</param>
    /// <returns>
    ///     Returns true if the coordinate is inside the perimeter
    /// </returns>
    public bool IsPointInside(Position coordinate)
    {
        // First, check if the coordinate is inside the area of the .asc file.
        // The comparison with "1" ensures that it's not a "non-walkable" cell (the area outside of our polygon).
        // It's the NoData value that was set previously in QGIS.
        return Extent.Contains(coordinate.X, coordinate.Y) && GetValue(coordinate) != 1;
    }
}