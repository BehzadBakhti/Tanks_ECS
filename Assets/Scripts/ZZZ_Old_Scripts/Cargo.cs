using Unity.Entities;

[GenerateAuthoringComponent]
public struct Cargo : IComponentData
{
    public float weight;
}


//public class CargoHandlingSystem : SystemBase
//{

//    protected override void OnUpdate()
//    {
//        var input = GetSingleton<InputCollectorComponent>();

//        Entities.ForEach((Entity entity, int entityInQueryIndex) => { });
//    }
//}


public struct Picked : IComponentData
{

}
