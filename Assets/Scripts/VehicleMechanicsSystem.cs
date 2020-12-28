using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(StepPhysicsWorld))]
public class VehicleMechanicsSystem : SystemBase
{
    BuildPhysicsWorld m_BuildPhysicsWorldSystem;

    protected override void OnCreate()
    {
        m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
        RequireForUpdate(GetEntityQuery(new EntityQueryDesc
        {
            All = new ComponentType[] { typeof(VehicleConfiguration) }
        }));
    }

    protected override void OnUpdate()
    {
        Dependency = m_BuildPhysicsWorldSystem.GetOutputDependency();

        // update vehicle properties first
        Dependency = Entities
            .WithName("PrepareVehiclesJob")
            .WithBurst()
            .ForEach((
                Entity entity, ref VehicleBody vehicleBody,
                in VehicleConfiguration mechanics, in PhysicsMass mass, in Translation translation, in Rotation rotation
            ) =>
            {
                vehicleBody.WorldCenterOfMass = mass.GetCenterOfMassWorldSpace(in translation, in rotation);

                // calculate a simple slip factor based on chassis tilt
                float3 worldUp = math.mul(rotation.Value, math.up());
                vehicleBody.SlopeSlipFactor = math.pow(math.abs(math.dot(worldUp, math.up())), 4f);
            })
            .Schedule(Dependency);

        Dependency.Complete();

        // this sample makes direct modifications to impulses between BuildPhysicsWorld and StepPhysicsWorld
        // we thus use PhysicsWorldExtensions rather than modifying component data, since they have already been consumed by BuildPhysicsWorld
        PhysicsWorld world = m_BuildPhysicsWorldSystem.PhysicsWorld;
        var collisionWorld = world.CollisionWorld;

        // update each wheel
        var commandBuffer = new EntityCommandBuffer(Allocator.TempJob);
        Entities
            .WithName("VehicleWheelsJob")
            .WithBurst()
            .WithReadOnly(collisionWorld)
            .ForEach((
                Entity entity, in Translation localPosition, in Rotation localRotation, in Wheel wheel
            ) =>
            {
                Entity ce = wheel.Vehicle;
                if (ce == Entity.Null) return;
                int ceIdx = world.GetRigidBodyIndex(ce);
                if (-1 == ceIdx || ceIdx >= world.NumDynamicBodies) return;

                var mechanics = GetComponent<VehicleConfiguration>(ce);
                var vehicleBody = GetComponent<VehicleBody>(ce);

                float3 cePosition = GetComponent<Translation>(ce).Value;
                quaternion ceRotation = GetComponent<Rotation>(ce).Value;
                float3 ceCenterOfMass = vehicleBody.WorldCenterOfMass;
                float3 ceUp = math.mul(ceRotation, new float3(0f, 1f, 0f));
                float3 ceForward = math.mul(ceRotation, new float3(0f, 0f, 1f));
                float3 ceRight = math.mul(ceRotation, new float3(1f, 0f, 0f));

                CollisionFilter filter = world.GetCollisionFilter(ceIdx);

                float driveDesiredSpeed = 0f;
                bool driveEngaged = false;
                if (HasComponent<VehicleSpeed>(ce))
                {
                    var vehicleSpeed = GetComponent<VehicleSpeed>(ce);
                    driveDesiredSpeed = vehicleSpeed.DesiredSpeed;
                    driveEngaged = vehicleSpeed.DriveEngaged != 0;
                }

                float desiredSteeringAngle = HasComponent<VehicleSteering>(ce)
                    ? GetComponent<VehicleSteering>(ce).DesiredSteeringAngle
                    : 0f;

                RigidTransform worldFromChassis = new RigidTransform
                {
                    pos = cePosition,
                    rot = ceRotation
                };

                RigidTransform suspensionFromWheel = new RigidTransform
                {
                    pos = localPosition.Value,
                    rot = localRotation.Value
                };

                RigidTransform chassisFromWheel = math.mul(wheel.ChassisFromSuspension, suspensionFromWheel);
                RigidTransform worldFromLocal = math.mul(worldFromChassis, chassisFromWheel);

                // create a raycast from the suspension point on the chassis
                var worldFromSuspension = math.mul(worldFromChassis, wheel.ChassisFromSuspension);
                float3 rayStart = worldFromSuspension.pos;
                float3 rayEnd = (-ceUp * (mechanics.suspensionLength + mechanics.wheelBase)) + rayStart;

                if (mechanics.drawDebugInformation != 0)
                    Debug.DrawRay(rayStart, rayEnd - rayStart);

                var raycastInput = new RaycastInput
                {
                    Start = rayStart,
                    End = rayEnd,
                    Filter = filter
                };

                var hit = world.CastRay(raycastInput, out var rayResult);

                var invWheelCount = mechanics.invWheelCount;

                // Calculate a simple slip factor based on chassis tilt.
                float slopeSlipFactor = vehicleBody.SlopeSlipFactor;

                float3 wheelPos = math.select(raycastInput.End, rayResult.Position, hit);
                wheelPos -= (cePosition - ceCenterOfMass);

                float3 velocityAtWheel = world.GetLinearVelocity(ceIdx, wheelPos);

                float3 weUp = ceUp;
                float3 weRight = ceRight;
                float3 weForward = ceForward;

                // Assumed hierarchy:
                // - chassis
                //  - mechanics
                //   - suspension
                //    - wheel (rotates about yaw axis and translates along suspension up)
                //     - graphic (rotates about pitch axis)

                #region handle wheel steering
                {
                    // update yaw angle if wheel is used for steering
                    if (wheel.UsedForSteering != 0)
                    {
                        quaternion wRotation = quaternion.AxisAngle(ceUp, desiredSteeringAngle);
                        weRight = math.rotate(wRotation, weRight);
                        weForward = math.rotate(wRotation, weForward);

                        commandBuffer.SetComponent(entity, new Rotation { Value = quaternion.AxisAngle(math.up(), desiredSteeringAngle) });
                    }
                }
                #endregion

                float currentSpeedUp = math.dot(velocityAtWheel, weUp);
                float currentSpeedForward = math.dot(velocityAtWheel, weForward);
                float currentSpeedRight = math.dot(velocityAtWheel, weRight);

                #region handle wheel rotation
                {
                    // update rotation of graphical representation about axle
                    bool isDriven = driveEngaged && wheel.UsedForDriving != 0;
                    float weRotation = isDriven
                        ? (driveDesiredSpeed / mechanics.wheelBase)
                        : (currentSpeedForward / mechanics.wheelBase);

                    weRotation = math.radians(weRotation);
                    var currentRotation = GetComponent<Rotation>(wheel.GraphicalRepresentation).Value;
                    commandBuffer.SetComponent(wheel.GraphicalRepresentation, new Rotation
                    {
                        // assumes wheels are aligned with chassis in "bind pose"
                        Value = math.mul(currentRotation, quaternion.AxisAngle(new float3(1f, 0f, 0f), weRotation))
                    });
                }
                #endregion

                var parentFromWorld = math.inverse(worldFromSuspension);
                if (!hit)
                {
                    float3 wheelDesiredPos = (-ceUp * mechanics.suspensionLength) + rayStart;
                    var worldPosition = math.lerp(worldFromLocal.pos, wheelDesiredPos, mechanics.suspensionDamping / mechanics.suspensionStrength);
                    // update translation of wheels along suspension column
                    commandBuffer.SetComponent(entity, new Translation
                    {
                        Value = math.mul(parentFromWorld, new float4(worldPosition, 1f)).xyz
                    });
                }
                else
                {
                    // remove the wheelbase to get wheel position.
                    float fraction = rayResult.Fraction - (mechanics.wheelBase) / (mechanics.suspensionLength + mechanics.wheelBase);

                    float3 wheelDesiredPos = math.lerp(rayStart, rayEnd, fraction);
                    // update translation of wheels along suspension column
                    var worldPosition = math.lerp(worldFromLocal.pos, wheelDesiredPos, mechanics.suspensionDamping / mechanics.suspensionStrength);
                    commandBuffer.SetComponent(entity, new Translation
                    {
                        Value = math.mul(parentFromWorld, new float4(worldPosition, 1f)).xyz
                    });

                    #region Suspension
                    {
                        // Calculate and apply the impulses
                        var posA = rayEnd;
                        var posB = rayResult.Position;
                        var lvA = currentSpeedUp * weUp;
                        var lvB = world.GetLinearVelocity(rayResult.RigidBodyIndex, posB);

                        var impulse = mechanics.suspensionStrength * (posB - posA) + mechanics.suspensionDamping * (lvB - lvA);
                        impulse = impulse * invWheelCount;
                        float impulseUp = math.dot(impulse, weUp);

                        // Suspension shouldn't necessarily pull the vehicle down!
                        float downForceLimit = -0.25f;
                        if (downForceLimit < impulseUp)
                        {
                            impulse = impulseUp * weUp;

                            UnityEngine.Assertions.Assert.IsTrue(math.all(math.isfinite(impulse)));
                            world.ApplyImpulse(ceIdx, impulse, posA);

                            if (mechanics.drawDebugInformation != 0)
                                Debug.DrawRay(wheelDesiredPos, impulse, Color.green);
                        }
                    }
                    #endregion

                    #region Sideways friction
                    {
                        float deltaSpeedRight = (0.0f - currentSpeedRight);
                        deltaSpeedRight = math.clamp(deltaSpeedRight, -mechanics.wheelMaxImpulseRight, mechanics.wheelMaxImpulseRight);
                        deltaSpeedRight *= mechanics.wheelFrictionRight;
                        deltaSpeedRight *= slopeSlipFactor;

                        float3 impulse = deltaSpeedRight * weRight;
                        float effectiveMass = world.GetEffectiveMass(ceIdx, impulse, wheelPos);
                        impulse = impulse * effectiveMass * invWheelCount;

                        UnityEngine.Assertions.Assert.IsTrue(math.all(math.isfinite(impulse)));
                        world.ApplyImpulse(ceIdx, impulse, wheelPos);
                        world.ApplyImpulse(rayResult.RigidBodyIndex, -impulse, wheelPos);

                        if (mechanics.drawDebugInformation != 0)
                            Debug.DrawRay(wheelDesiredPos, impulse, Color.red);
                    }
                    #endregion

                    #region Drive
                    {
                        if (driveEngaged && wheel.UsedForDriving != 0)
                        {
                            float deltaSpeedForward = (driveDesiredSpeed - currentSpeedForward);
                            deltaSpeedForward = math.clamp(deltaSpeedForward, -mechanics.wheelMaxImpulseForward, mechanics.wheelMaxImpulseForward);
                            deltaSpeedForward *= mechanics.wheelFrictionForward;
                            deltaSpeedForward *= slopeSlipFactor;

                            float3 impulse = deltaSpeedForward * weForward;

                            float effectiveMass = world.GetEffectiveMass(ceIdx, impulse, wheelPos);
                            impulse = impulse * effectiveMass * invWheelCount;

                            UnityEngine.Assertions.Assert.IsTrue(math.all(math.isfinite(impulse)));
                            world.ApplyImpulse(ceIdx, impulse, wheelPos);
                            world.ApplyImpulse(rayResult.RigidBodyIndex, -impulse, wheelPos);

                            if (mechanics.drawDebugInformation != 0)
                                Debug.DrawRay(wheelDesiredPos, impulse, Color.blue);
                        }
                    }
                    #endregion
                }
            })
            .Run();

        commandBuffer.Playback(EntityManager);
        commandBuffer.Dispose();
    }
}