using Unity.Entities;

namespace Assets.Scripts.TankController
{
    [GenerateAuthoringComponent]
    public struct TankSpec : IComponentData
    {
        public float power;
        public float breakPower;
        public float weight;
        public float turnSpeed;
        public float maxSpeed;
        public float dragCoefficient;


    }
}