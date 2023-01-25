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

    /// <summary>
    ///     Update the nearest flower to the agent
    /// </summary>
    /// <exception cref="NotImplementedException"></exception>
    private void UpdateNearestFlower()
    {
        foreach (var flower in flowerArea.Flowers)
            if (nearestFlower == null && flower.HasNectar)
            {
                // No current flower and this flower has nectar, so set to this flower
                nearestFlower = flower;
            }
            else if (flower.HasNectar)
            {
                // Calculate distance to this flower and distance to the current nearest flower
                var distanceToFlower = Vector3.Distance(flower.transform.position, beakTip.position);
                var distanceToCurrentNearestFlower =
                    Vector3.Distance(nearestFlower.transform.position, beakTip.position);

                // If current nearest flower is empty OR this flower is closer, update the nearest flower
                if (!nearestFlower.HasNectar || distanceToFlower < distanceToCurrentNearestFlower)
                    nearestFlower = flower;
            }
    }

    /// <summary>
    ///     Move the agent to a safe random position (i.e. does not collide with anything0)
    ///     If in front of flower, also point the beat at the flower
    /// </summary>
    /// <param name="inFrontOfFlower">Whether to choose a spot in front of a flower</param>
    /// <exception cref="NotImplementedException"></exception>
    private void MoveToSafeRandomPosition(bool inFrontOfFlower)
    {
        var safePositionFound = false;
        var attemptsRemaining = 100; // Prevent an infinite loop
        var potentialPosition = Vector3.zero;
        var potentialRotation = new Quaternion();

        // Loop until a safe position is found or we run out of attempts
        while (!safePositionFound && attemptsRemaining > 0)
        {
            attemptsRemaining--;
            if (inFrontOfFlower)
            {
                // Pick a random flower
                var randomFlower = flowerArea.Flowers[Random.Range(0, flowerArea.Flowers.Count)];

                // Position 10 to 20 cm in from of the flower
                var distanceFromFlower = Random.Range(.1f, .2f);
                potentialPosition = randomFlower.transform.position + randomFlower.FlowerUpVector * distanceFromFlower;

                // Point beak at flower (bird's head is center of transform)
                var toFlower = randomFlower.FlowerCenterPosition - potentialPosition;
                potentialRotation = Quaternion.LookRotation(toFlower, Vector3.up);
            }
            else
            {
                // Pick a random height from the ground
                var height = Random.Range(1.2f, 2.5f);

                // Pick a random radius from the center of the area
                var radius = Random.Range(2f, 7f);

                // Pick a random direction rotated around the y axis
                var direction = Quaternion.Euler(0f, Random.Range(-180f, 180f), 0f);

                // Combine height, radius and direction to pick a potential position
                potentialPosition = flowerArea.transform.position + Vector3.up * height +
                                    direction * Vector3.forward * radius;

                // Choose and set random position pitch and yaw
                var pitch = Random.Range(-60f, 60f);
                var yaw = Random.Range(-180f, 180f);
                potentialRotation = Quaternion.Euler(pitch, yaw, 0f);
            }

            // Check to see if the agent will collide with anything
            var colliders = Physics.OverlapSphere(potentialPosition, 0.05f);

            // Safe position has been found if no colliders are overlapped
            safePositionFound = colliders.Length = 0;
        }

        Debug.Assert(safePositionFound, "Could not find a safe position to spawn");

        // Set the position and rotation
        transform.position = potentialPosition;
        transform.rotation = potentialRotation;
    }
}