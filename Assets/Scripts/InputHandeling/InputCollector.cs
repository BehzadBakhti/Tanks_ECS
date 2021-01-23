using Unity.Entities;
using UnityEngine;

public class InputCollector : MonoBehaviour
{
    public EntityManager _manager;
    public Entity _inputCollection;
    private void Awake()
    {
        _manager = World.DefaultGameObjectInjectionWorld.EntityManager;
        _inputCollection = _manager.CreateEntity();
        _manager.AddComponent<InputCollectorComponent>(_inputCollection);
    }

    private void Update()
    {
        var collector = _manager.GetComponentData<InputCollectorComponent>(_inputCollection);
        collector.anyKeyDown = Input.anyKeyDown;

        collector.space = Input.GetKey(KeyCode.Space);
        collector.mouseDown0 = Input.GetMouseButtonDown(0) ? collector.mouseDown0 + 1 : collector.mouseDown0;
        collector.mouseDown1 = Input.GetMouseButtonDown(1) ? collector.mouseDown1 + 1 : collector.mouseDown1;
        collector.mouseUp0 = Input.GetMouseButtonUp(0) ? collector.mouseUp0 + 1 : collector.mouseUp0;
        collector.mouseUp1 = Input.GetMouseButtonUp(1) ? collector.mouseUp1 + 1 : collector.mouseUp1;



        collector.horizontal = Input.GetAxis("Horizontal");
        collector.vertical = Input.GetAxis("Vertical");
        var rightNeg = Input.GetKey(KeyCode.J) ? -1 : 0;
        var rightPositive = Input.GetKey(KeyCode.L) ? 1 : 0;
        collector.rightHorizontal = rightPositive + rightNeg;

        var rightDown = Input.GetKey(KeyCode.J) ? -1 : 0;
        var rightUp = Input.GetKey(KeyCode.L) ? 1 : 0;
        collector.rightVertical = rightDown + rightUp;
        collector.mouse0 = Input.GetMouseButton(0);
        collector.mouse1 = Input.GetMouseButton(1);

        _manager.SetComponentData(_inputCollection, collector);
    }
}