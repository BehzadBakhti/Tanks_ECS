using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

class VehicleAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
#pragma warning disable 649
    public bool ActiveAtStart;

    [Header("Handling")]
    public float TopSpeed = 10.0f;
    public float MaxSteeringAngle = 30.0f;
    [Range(0f, 1f)] public float SteeringDamping = 0.1f;
    [Range(0f, 1f)] public float SpeedDamping = 0.01f;


   

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


    }
}