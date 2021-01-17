using Unity.Collections;
using Unity.Entities;
using UnityEngine;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup), OrderLast = true)]

public class InputResetSystem : SystemBase
{
    private NativeArray<bool> _isDragging;

    protected override void OnCreate()
    {
        base.OnCreate();
        _isDragging = new NativeArray<bool>(1, Allocator.Persistent);
    }

    protected override void OnUpdate()
    {
        var isDragging = _isDragging;

        Entities.ForEach((ref InputCollectorComponent inputCollector) =>
        {
            isDragging[0] = inputCollector.mouse0;

            inputCollector = new InputCollectorComponent { mouse0 = isDragging[0] };
        }).ScheduleParallel();
    }

    protected override void OnDestroy()
    {
        if (_isDragging != null && _isDragging.IsCreated)
            _isDragging.Dispose();
    }
}


public struct InputCollectorComponent : IComponentData
{
    public bool mouse1, mouse0, anyKeyDown, leftShift, rightShift, leftCtrl, rightCtrl;
    public int q, e, f, g, h;

    // public int alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7, alpha8, alpha9;
    
    public int mouseDown0, mouseUp0;
    public int mouseDown1, mouseUp1;
    public bool space;
    public float horizontal, vertical, mouseX, mouseY;
}



