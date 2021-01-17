using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using Unity.Physics.Systems;
using UnityEngine;

namespace Assets.Scripts.TankController
{
    [GenerateAuthoringComponent]
    public struct RayCasterPoint : IComponentData
    {
        public float castLength;
        public float3 stablePosition;
        public float3 stableUpsideVector;
        public bool hasHit;
    }

    [UpdateBefore(typeof(TankAdaptPositionSystem))]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public class RayCasterPointPositionSystem : SystemBase
    {
        private BuildPhysicsWorld _buildPhysicsWorld;
        protected override void OnCreate()
        {
            base.OnCreate();
            _buildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
        }

        protected override void OnUpdate()
        {
            CollisionWorld collisionWorld = _buildPhysicsWorld.PhysicsWorld.CollisionWorld;
            Entities.WithReadOnly(collisionWorld).ForEach((ref RayCasterPoint rayCaster, in LocalToWorld l2w,in Rotation rot) =>
            {
                var rayInput = new RaycastInput
                {
                    Start = l2w.Position,
                    End = l2w.Position - rayCaster.castLength * l2w.Up,
                    Filter = new CollisionFilter()
                    {
                        BelongsTo = ~0u,
                        CollidesWith = ~0u,
                        GroupIndex = 0
                    }
                };
                Debug.DrawLine(l2w.Position, l2w.Position  - rayCaster.castLength * l2w.Up);
                rayCaster.hasHit = collisionWorld.CastRay(rayInput, out var hit);
              if (rayCaster.hasHit)
              {
                  rayCaster.stablePosition = hit.Position + hit.SurfaceNormal*0.2f;
                  rayCaster.stableUpsideVector = hit.SurfaceNormal;
                  Debug.DrawLine(hit.Position, hit.Position + hit.SurfaceNormal *0.2f,Color.red);

                }

            }).ScheduleParallel();
        }
    }
}