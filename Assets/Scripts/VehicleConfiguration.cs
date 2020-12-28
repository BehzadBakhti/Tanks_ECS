using Unity.Entities;

struct VehicleConfiguration : IComponentData
{
    public float wheelBase;
    public float wheelFrictionRight;
    public float wheelFrictionForward;
    public float wheelMaxImpulseRight;
    public float wheelMaxImpulseForward;
    public float suspensionLength;
    public float suspensionStrength;
    public float suspensionDamping;
    public float invWheelCount;
    public byte drawDebugInformation;
}