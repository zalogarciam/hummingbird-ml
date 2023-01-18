using Unity.MLAgents;
using UnityEngine;

/// <summary>
///     A hummingbird Machine Learning Agent
/// </summary>
public class HummingbirdAgent : Agent
{
    [Tooltip("The agent's camera")] public Camera agentCamera;

    [Tooltip("Transform at the tip of the beak")]
    public Transform beakTip;

    [Tooltip("Force to apply when moving")]
    public float moveForce = 2f;

    [Tooltip("Speed to pitch up or down")] public float pitchSpeed = 100f;

    [Tooltip("Whether this is training mode or gameplay mode")]
    public bool trainingMode;

    [Tooltip("Speed to rotate around the up axis")]
    public float yawSpeed = 100f;
}