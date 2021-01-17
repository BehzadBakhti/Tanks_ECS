using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Assets.Scripts.TankController
{
    [GenerateAuthoringComponent]
    public struct Tank : IComponentData
    {
    }

    [UpdateAfter(typeof(TankAdaptPositionSystem))]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public class TankMovementSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            var input = GetSingleton<InputCollectorComponent>();
            var inZ = input.vertical;
            var breaking = input.space;
            var dt = Time.DeltaTime;
            Entities.WithAll<Tank>().ForEach(
                (ref Translation pos, ref ObjVelocity velocity, in Rotation rot,
                    in TankSpec spec) =>
                {
                    velocity.value = math.length(velocity.value) * math.forward(rot.Value)*math.sign(math.dot(velocity.value,math.forward(rot.Value)));
                    var breakForce = breaking && math.length(velocity.value)>0 ? spec.breakPower * math.normalize(velocity.value) : 0;
                    var dragForce = math.length(velocity.value)>0?spec.dragCoefficient * math.pow(math.length(velocity.value), 2)*math.normalize(velocity.value):0;
                  //  Debug.Log(velocity.value);
                    var acc = (spec.power * inZ * math.forward(rot.Value) -dragForce-breakForce)/ spec.weight;

                    var newV= acc*dt + velocity.value;
                    if (math.length(newV) <0.5f && math.abs(inZ)<0.01f)
                    {
                        newV = float3.zero;
                    }

                    velocity.value = newV;
                    pos.Value += velocity.value * dt;

                }).ScheduleParallel();
        }
    }
    [UpdateAfter(typeof(TankAdaptPositionSystem))]
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public class TankRotationSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            var input = GetSingleton<InputCollectorComponent>();
            var inX = input.horizontal;
       
            var dt = Time.DeltaTime;
            Entities.WithAll<Tank>().ForEach(
                ( ref Rotation rot, ref ObjVelocity velocity,
                    in TankSpec spec) =>
                {
                    var signCoeff = math.sign(math.dot(velocity.value, math.forward(rot.Value))) >= 0 ? 1 : -1;
                    rot.Value = math.mul(math.normalize(rot.Value), quaternion.AxisAngle(math.up(), inX * dt* signCoeff * math.radians(spec.turnSpeed)));


                }).ScheduleParallel();
        }
    }





}