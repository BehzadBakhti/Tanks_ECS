using Unity.Entities;

namespace Assets.Scripts.TankController
{
    [GenerateAuthoringComponent]
    public struct TankGun : IComponentData
    {
        public float maxTemperature;
        public float temperature;
    }

    public struct AngleComponent : IComponentData
    {
        public float turnSpeed;
        public float maxAngle;
        public float angle;
    }
}