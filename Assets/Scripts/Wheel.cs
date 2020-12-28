using Unity.Entities;
using Unity.Mathematics;

struct Wheel : IComponentData
{
    public Entity Vehicle;
    public Entity GraphicalRepresentation;
    public byte UsedForSteering;
    public byte UsedForDriving;
    public RigidTransform ChassisFromSuspension;
}