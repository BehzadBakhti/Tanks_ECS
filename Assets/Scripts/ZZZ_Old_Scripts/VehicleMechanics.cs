using System.Collections.Generic;
using Unity.Physics.Authoring;
using UnityEngine;

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