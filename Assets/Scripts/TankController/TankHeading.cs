using System;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Assets.Scripts.TankController
{
    [GenerateAuthoringComponent]
    public struct TankHeading : IComponentData
    {
        public float maxDeviation;
        public float turnSpeed;
    }


    public class TankHeadingSystemControl : SystemBase
    {
        protected override void OnUpdate()
        {
            var input = GetSingleton<InputCollectorComponent>();
            var rightHorizontal = input.rightHorizontal;
            if (Math.Abs(rightHorizontal) < math.EPSILON) return;
            var dt = Time.DeltaTime;
            Entities.ForEach((ref Rotation rot, in TankHeading heading) =>
            {

                var angle = math.acos(math.dot(math.forward(rot.Value), new float3(0, 0, 1))) * 180 / math.PI;
                angle = math.cross(math.forward(rot.Value), new float3(0, 0, 1)).y > 0 ? -angle : angle;


                Debug.Log(angle);
                if ((rightHorizontal > 0 && angle < heading.maxDeviation)|| (rightHorizontal < 0 && angle > -heading.maxDeviation))
                    rot.Value = math.mul(math.normalize(rot.Value), quaternion.AxisAngle(math.up(), rightHorizontal * dt * math.radians(heading.turnSpeed)));
            }).ScheduleParallel();
        }
    }
}