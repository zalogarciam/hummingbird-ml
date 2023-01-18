using System;
using Unity.MLAgents;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
///     A hummingbird Machine Learning Agent
/// </summary>
public class HummingbirdAgent : Agent
{
    // Maximum angle that the bird can pitch up or down
    private const float MaxPitchAngle = 80f;

    // Maximum distance from the beak tip to accept nectar collision
    private const float BeakTipRadius = 0.008f;
    [Tooltip("The agent's camera")] public Camera agentCamera;

    [Tooltip("Transform at the tip of the beak")]
    public Transform beakTip;

    // The flower are that the agent is i n
    private FlowerArea flowerArea;

    // Whether the agent is frozen (intentionally not flying)
    private bool frozen = false;

    [Tooltip("Force to apply when moving")]
    public float moveForce = 2f;

    // The nearest flower to the agent
    private Flower nearestFlower;

    [Tooltip("Speed to pitch up or down")] public float pitchSpeed = 100f;

    // The rigid body of the agent
    private new Rigidbody rigidbody;

    // Allows for smoother pitch changes
    private float smoothPitchChange = 0f;

    // Allows for smoother yaw changes
    private float smoothYawChange = 0f;

    [Tooltip("Whether this is training mode or gameplay mode")]
    public bool trainingMode;

    [Tooltip("Speed to rotate around the up axis")]
    public float yawSpeed = 100f;

    /// <summary>
    ///     The amount of nectar the agent has obtained this episode
    /// </summary>
    public float NectarObtained { get; private set; }

    /// <summary>
    ///     Initialize the agent
    /// </summary>
    public override void Initialize()
    {
        rigidbody = GetComponent<Rigidbody>();
        flowerArea = GetComponentInParent<FlowerArea>();

        // If not training mode, no max step, play forever
        if (!trainingMode) MaxStep = 0;
    }

    /// <summary>
    ///     Reset the agent when an episode begins
    /// </summary>
    public override void OnEpisodeBegin()
    {
        if (trainingMode)
            // Only reset flowers in training when there is one agent per area
            flowerArea.ResetFlowers();

        // Reset nectar obtained
        NectarObtained = 0;

        // Zero out velocities so that movement stops before a new episode begins
        rigidbody.velocity = Vector3.zero;
        rigidbody.angularVelocity = Vector3.zero;

        // Default to spawning in front of a flower
        var inFrontOfFlower = true;
        if (trainingMode)
            // Spawn in front of flower 50% during training
            inFrontOfFlower = Random.value > 0.5f;

        // Move the agent to a new random position
        MoveToSafeRandomPosition(inFrontOfFlower);

        // Recalculate the nearest flower now that the agent has moved
        UpdateNearestFlower();
    }

    private void UpdateNearestFlower()
    {
        throw new NotImplementedException();
    }

    private void MoveToSafeRandomPosition(bool inFrontOfFlower)
    {
        throw new NotImplementedException();
    }
}