using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Assets.Scripts.TankController
{
 [GenerateAuthoringComponent]
    public struct TankStructure : IComponentData
    {
        public Entity center;
        public Entity frontPoint;
        public Entity backPoint;
        public Entity leftPoint;
        public Entity rightPoint;
    }

    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public class TankAdaptPositionSystem : SystemBase
    {
        private BuildPhysicsWorld _buildPhysicsWorld;
        protected override void OnCreate()
        {
            base.OnCreate();
            _buildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
        }

        protected override void OnUpdate()
        {
            var rayCasters = GetComponentDataFromEntity<RayCasterPoint>(true);
            var dt = Time.DeltaTime;
            Entities.WithReadOnly(rayCasters).ForEach((ref TankStructure tank, ref Translation pos, ref Rotation rot) =>
            {
               
                var forwardVect = float3.zero;
                var upsideVect = float3.zero;
                if (rayCasters[tank.frontPoint].hasHit && rayCasters[tank.backPoint].hasHit)
                {
                    //if(!tank.center.hasHit)
                    //{
                    forwardVect = rayCasters[tank.frontPoint].stablePosition - rayCasters[tank.backPoint].stablePosition;
                    pos.Value = math.lerp(pos.Value,(rayCasters[tank.backPoint].stablePosition +
                                 rayCasters[tank.frontPoint].stablePosition +
                                 rayCasters[tank.leftPoint].stablePosition +
                                 rayCasters[tank.rightPoint].stablePosition) / 4,1);
                    upsideVect= (rayCasters[tank.backPoint].stableUpsideVector +
                                 rayCasters[tank.frontPoint].stableUpsideVector +
                                 rayCasters[tank.leftPoint].stableUpsideVector +
                                 rayCasters[tank.rightPoint].stableUpsideVector) / 4;
                    rot.Value = quaternion.LookRotation(math.normalize(math.lerp(math.forward(rot.Value),forwardVect,5*dt)), math.normalize(upsideVect));
                    //}
                    //else
                    //{

                    //}
                }



            }).ScheduleParallel();
        }
    }

}