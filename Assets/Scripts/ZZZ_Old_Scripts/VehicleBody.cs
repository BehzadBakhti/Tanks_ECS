using Unity.Entities;
using Unity.Mathematics;

struct VehicleBody : IComponentData
{
    public float SlopeSlipFactor;
    public float3 WorldCenterOfMass;
}