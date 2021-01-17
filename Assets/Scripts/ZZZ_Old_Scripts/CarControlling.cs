using Unity.Entities;

[GenerateAuthoringComponent]
public struct CarControlling : IComponentData
{
    public float motorForce;
    public float brakeForce;
    public float maxSteerAngle;
    public float steerability;
}