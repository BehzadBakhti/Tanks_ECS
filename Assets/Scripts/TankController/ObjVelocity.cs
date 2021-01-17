using Unity.Entities;
using Unity.Mathematics;

namespace Assets.Scripts.TankController
{
    [GenerateAuthoringComponent]
    public struct ObjVelocity : IComponentData
    {
        public float3 value;
    }
}