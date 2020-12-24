
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;
using UnityEngine;

public class CarSpecAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public float emptyWeight;
    public float dragCoef;
    public float topSpeed;
    public GameObject[] wheels;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {

        var list = new FixedList128<Entity>();
        for (int i = 0; i < wheels.Length; i++)
        {
            list.Add(conversionSystem.GetPrimaryEntity(wheels[i]));
        }

        dstManager.AddComponentData(entity, new CarSpec()
        {
            emptyWeight = emptyWeight,
            dragCoef = dragCoef,
            topSpeed = topSpeed,
            wheels = list,
        });
    }
}





    [RequireComponent(typeof(PhysicsBodyAuthoring))]
    public class VehicleMechanics : MonoBehaviour
    {
        [Header("Wheel0 Parameters...")]
        public List<GameObject> wheels = new List<GameObject>();
        public float wheelBase = 0.5f;
        public float wheelFrictionRight = 0.5f;
        public float wheelFrictionForward = 0.5f;
        public float wheelMaxImpulseRight = 10.0f;
        public float wheelMaxImpulseForward = 10.0f;
        [Header("Suspension Parameters...")]
        public float suspensionLength = 0.5f;
        public float suspensionStrength = 1.0f;
        public float suspensionDamping = 0.1f;
        [Header("Steering Parameters...")]
        public List<GameObject> steeringWheels = new List<GameObject>();
        [Header("Drive Parameters...")]
        public List<GameObject> driveWheels = new List<GameObject>();
        [Header("Miscellaneous Parameters...")]
        public bool drawDebugInformation = false;
    }

    // ensure built-in physics conversion systems have run
    [UpdateAfter(typeof(EndColliderConversionSystem))]
    [UpdateAfter(typeof(PhysicsBodyConversionSystem))]
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

    // configuration properties of the vehicle mechanics, which change with low frequency at run-time
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

    // physics properties of the vehicle rigid body, which change with high frequency at run-time
    struct VehicleBody : IComponentData
    {
        public float SlopeSlipFactor;
        public float3 WorldCenterOfMass;
    }

    struct Wheel : IComponentData
    {
        public Entity Vehicle;
        public Entity GraphicalRepresentation;
        public byte UsedForSteering;
        public byte UsedForDriving;
        public RigidTransform ChassisFromSuspension;
    }

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


struct Vehicle : IComponentData { }

struct VehicleSpeed : IComponentData
{
    public float TopSpeed;
    public float DesiredSpeed;
    public float Damping;
    public byte DriveEngaged;
}

struct VehicleSteering : IComponentData
{
    public float MaxSteeringAngle;
    public float DesiredSteeringAngle;
    public float Damping;
}

enum VehicleCameraOrientation
{
    Absolute,
    Relative
}

struct VehicleCameraSettings : IComponentData
{
    public VehicleCameraOrientation OrientationType;
    public float OrbitAngularSpeed;
}

struct VehicleCameraReferences : IComponentData
{
    public Entity CameraOrbit;
    public Entity CameraTarget;
    public Entity CameraTo;
    public Entity CameraFrom;
}

class VehicleAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
#pragma warning disable 649
    public bool ActiveAtStart;

    [Header("Handling")]
    public float TopSpeed = 10.0f;
    public float MaxSteeringAngle = 30.0f;
    [Range(0f, 1f)] public float SteeringDamping = 0.1f;
    [Range(0f, 1f)] public float SpeedDamping = 0.01f;

    [Header("Camera Settings")]
    public Transform CameraOrbit;
    public VehicleCameraOrientation CameraOrientation = VehicleCameraOrientation.Relative;
    public float CameraOrbitAngularSpeed = 180f;
    public Transform CameraTarget;
    public Transform CameraTo;
    public Transform CameraFrom;
#pragma warning restore 649

    void OnValidate()
    {
        TopSpeed = math.max(0f, TopSpeed);
        MaxSteeringAngle = math.max(0f, MaxSteeringAngle);
        SteeringDamping = math.clamp(SteeringDamping, 0f, 1f);
        SpeedDamping = math.clamp(SpeedDamping, 0f, 1f);
    }

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        if (ActiveAtStart)
            dstManager.AddComponent<ActiveVehicle>(entity);

        dstManager.AddComponent<Vehicle>(entity);

        dstManager.AddComponentData(entity, new VehicleCameraSettings
        {
            OrientationType = CameraOrientation,
            OrbitAngularSpeed = math.radians(CameraOrbitAngularSpeed)
        });

        dstManager.AddComponentData(entity, new VehicleSpeed
        {
            TopSpeed = TopSpeed,
            Damping = SpeedDamping
        });

        dstManager.AddComponentData(entity, new VehicleSteering
        {
            MaxSteeringAngle = math.radians(MaxSteeringAngle),
            Damping = SteeringDamping
        });

        dstManager.AddComponentData(entity, new VehicleCameraReferences
        {
            CameraOrbit = conversionSystem.GetPrimaryEntity(CameraOrbit),
            CameraTarget = conversionSystem.GetPrimaryEntity(CameraTarget),
            CameraTo = conversionSystem.GetPrimaryEntity(CameraTo),
            CameraFrom = conversionSystem.GetPrimaryEntity(CameraFrom)
        });
    }
}


struct VehicleInput : IComponentData
{
    public float2 Looking;
    public float2 Steering;
    public float Throttle;
    public int Change; // positive to change to a subsequent vehicle, negative to change to a previous one
}

[UpdateInGroup(typeof(InitializationSystemGroup))]
class VehicleInputHandlingSystem : SystemBase
{
    protected override void OnUpdate()
    {
        var input = GetSingleton<VehicleInput>();

        Entities
            .WithName("ActiveVehicleInputHandlingJob")
            .WithoutBurst()
            .WithStructuralChanges()
            .WithAll<ActiveVehicle>()
            .ForEach((ref VehicleSpeed speed, ref VehicleSteering steering, in VehicleCameraSettings cameraSettings, in VehicleCameraReferences references) =>
            {
                float x = input.Steering.x;
                float a = input.Throttle;
                float z = input.Looking.x;

                var newSpeed = a * speed.TopSpeed;
                speed.DriveEngaged = (byte)(newSpeed == 0f ? 0 : 1);
                speed.DesiredSpeed = math.lerp(speed.DesiredSpeed, newSpeed, speed.Damping);

                var newSteeringAngle = x * steering.MaxSteeringAngle;
                steering.DesiredSteeringAngle = math.lerp(steering.DesiredSteeringAngle, newSteeringAngle, steering.Damping);

                if (HasComponent<Rotation>(references.CameraOrbit))
                {
                    var orientation = GetComponent<Rotation>(references.CameraOrbit);
                    switch (cameraSettings.OrientationType)
                    {
                        case VehicleCameraOrientation.Relative:
                            orientation.Value = math.mul(orientation.Value, quaternion.Euler(0f, z * Time.DeltaTime * cameraSettings.OrbitAngularSpeed, 0f));
                            break;
                        case VehicleCameraOrientation.Absolute:
                            float4x4 worldFromLocal = HasComponent<LocalToWorld>(references.CameraOrbit)
                                ? GetComponent<LocalToWorld>(references.CameraOrbit).Value
                                : float4x4.identity;
                            float4x4 worldFromParent = HasComponent<LocalToParent>(references.CameraOrbit)
                                ? math.mul(worldFromLocal, math.inverse(GetComponent<LocalToParent>(references.CameraOrbit).Value))
                                : worldFromLocal;
                            var worldOrientation = quaternion.Euler(0f, z * math.PI, 0f);
                            orientation.Value = new quaternion(math.mul(worldFromParent, new float4x4(worldOrientation, float3.zero)));
                            break;
                    }
                    SetComponent(references.CameraOrbit, orientation);
                }
            }).Run();
    }
}



struct ActiveVehicle : IComponentData { }

class ChangeActiveVehicleSystem : SystemBase
{
    struct AvailableVehicle : ISystemStateComponentData { }

    EntityQuery m_ActiveVehicleQuery;
    EntityQuery m_VehicleInputQuery;

    EntityQuery m_NewVehicleQuery;
    EntityQuery m_ExistingVehicleQuery;
    EntityQuery m_DeletedVehicleQuery;

    NativeList<Entity> m_AllVehicles;

    protected override void OnCreate()
    {
        m_ActiveVehicleQuery = GetEntityQuery(typeof(ActiveVehicle), typeof(Vehicle));
        m_VehicleInputQuery = GetEntityQuery(typeof(VehicleInput));
        m_NewVehicleQuery = GetEntityQuery(new EntityQueryDesc
        {
            All = new[] { ComponentType.ReadOnly<Vehicle>() },
            None = new[] { ComponentType.ReadOnly<AvailableVehicle>() }
        });
        m_ExistingVehicleQuery = GetEntityQuery(new EntityQueryDesc
        {
            All = new[] { ComponentType.ReadOnly<Vehicle>(), ComponentType.ReadOnly<AvailableVehicle>() }
        });
        m_DeletedVehicleQuery = GetEntityQuery(new EntityQueryDesc
        {
            All = new[] { ComponentType.ReadOnly<AvailableVehicle>() },
            None = new[] { ComponentType.ReadOnly<Vehicle>() }
        });

        m_AllVehicles = new NativeList<Entity>(Allocator.Persistent);
    }

    protected override void OnDestroy() => m_AllVehicles.Dispose();

    protected override void OnUpdate()
    {
        // update stable list of vehicles if they have changed
        if (m_NewVehicleQuery.CalculateEntityCount() > 0 || m_DeletedVehicleQuery.CalculateEntityCount() > 0)
        {
            EntityManager.AddComponent(m_NewVehicleQuery, typeof(AvailableVehicle));
            EntityManager.RemoveComponent<AvailableVehicle>(m_DeletedVehicleQuery);

            m_AllVehicles.Clear();
            using (var allVehicles = m_ExistingVehicleQuery.ToEntityArray(Allocator.TempJob))
                m_AllVehicles.AddRange(allVehicles);
        }

        // do nothing if there are no vehicles
        if (m_AllVehicles.Length == 0)
            return;

        // validate active vehicle singleton
        var activeVehicle = Entity.Null;
        if (m_ActiveVehicleQuery.CalculateEntityCount() == 1)
            activeVehicle = m_ActiveVehicleQuery.GetSingletonEntity();
        else
        {
            using (var activeVehicles = m_ActiveVehicleQuery.ToEntityArray(Allocator.TempJob))
            {
                Debug.LogWarning(
                    $"Expected exactly one {nameof(VehicleAuthoring)} component in the scene to be marked {nameof(VehicleAuthoring.ActiveAtStart)}. " +
                    "First available vehicle is being set to active."
                );

                // prefer the first vehicle prospectively marked as active
                if (activeVehicles.Length > 0)
                {
                    activeVehicle = activeVehicles[0];
                    EntityManager.RemoveComponent<ActiveVehicle>(m_AllVehicles);
                }
                // otherwise use the first vehicle found
                else
                    activeVehicle = m_AllVehicles[0];

                EntityManager.AddComponent(activeVehicle, typeof(ActiveVehicle));
            }
        }

        // do nothing else if there are no vehicles to change to or if there is no input to change vehicle
        if (m_AllVehicles.Length < 2)
            return;

        var input = m_VehicleInputQuery.GetSingleton<VehicleInput>();

        if (input.Change == 0)
            return;

        // find the index of the currently active vehicle
        var activeVehicleIndex = 0;
        for (int i = 0, count = m_AllVehicles.Length; i < count; ++i)
        {
            if (m_AllVehicles[i] == activeVehicle)
                activeVehicleIndex = i;
        }

        // if the active vehicle index has actually changed, then move the active vehicle tag to the new vehicle
        var numVehicles = m_AllVehicles.Length;
        var newVehicleIndex = ((activeVehicleIndex + input.Change) % numVehicles + numVehicles) % numVehicles;
        if (newVehicleIndex == activeVehicleIndex)
            return;

        EntityManager.RemoveComponent<ActiveVehicle>(m_AllVehicles);
        EntityManager.AddComponent(m_AllVehicles[newVehicleIndex], typeof(ActiveVehicle));
    }
}
