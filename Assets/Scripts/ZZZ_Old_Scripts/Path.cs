
using Unity.Collections;
using Unity.Entities;

[GenerateAuthoringComponent]
public struct Path : IComponentData
{
    public FixedList128<WayPoint> wayPoints;
}


