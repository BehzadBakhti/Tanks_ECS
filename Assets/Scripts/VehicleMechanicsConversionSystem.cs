using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;

[UpdateAfter(typeof(EndColliderConversionSystem))]
[UpdateAfter(typeof(PhysicsBodyConversionSystem))]
//[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
class VehicleMechanicsConversionSystem : GameObjectConversionSystem
{
    protected override void OnUpdate()
    {
        Entities.ForEach((VehicleMechanics m) =>
        {
            var entity = GetPrimaryEntity(m);

            foreach (var wheel in m.wheels)
            {
                var wheelEntity = GetPrimaryEntity(wheel);

                // Assumed hierarchy:
                // - chassis
                //  - mechanics
                //   - suspension
                //    - wheel (rotates about yaw axis and translates along suspension up)
                //     - graphic (rotates about pitch axis)

                RigidTransform worldFromSuspension = new RigidTransform
                {
                    pos = wheel.transform.parent.position,
                    rot = wheel.transform.parent.rotation
                };

                RigidTransform worldFromChassis = new RigidTransform
                {
                    pos = wheel.transform.parent.parent.parent.position,
                    rot = wheel.transform.parent.parent.parent.rotation
                };

                var chassisFromSuspension = math.mul(math.inverse(worldFromChassis), worldFromSuspension);

                DstEntityManager.AddComponentData(wheelEntity, new Wheel
                {
                    Vehicle = entity,
                    GraphicalRepresentation = GetPrimaryEntity(wheel.transform.GetChild(0)), // assume wheel has a single child with rotating graphic
                    // TODO assume for now that driving/steering wheels also appear in this list
                    UsedForSteering = (byte)(m.steeringWheels.Contains(wheel) ? 1 : 0),
                    UsedForDriving = (byte)(m.driveWheels.Contains(wheel) ? 1 : 0),
                    ChassisFromSuspension = chassisFromSuspension
                });
            }

            DstEntityManager.AddComponent<VehicleBody>(entity);
            DstEntityManager.AddComponentData(entity, new VehicleConfiguration
            {
                wheelBase = m.wheelBase,
                wheelFrictionRight = m.wheelFrictionRight,
                wheelFrictionForward = m.wheelFrictionForward,
                wheelMaxImpulseRight = m.wheelMaxImpulseRight,
                wheelMaxImpulseForward = m.wheelMaxImpulseForward,
                suspensionLength = m.suspensionLength,
                suspensionStrength = m.suspensionStrength,
                suspensionDamping = m.suspensionDamping,
                invWheelCount = 1f / m.wheels.Count,
                drawDebugInformation = (byte)(m.drawDebugInformation ? 1 : 0)
            });
        });
    }
}