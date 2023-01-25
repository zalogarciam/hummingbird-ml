using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
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
    private bool frozen;

    [Tooltip("Force to apply when moving")]
    public float moveForce = 2f;

    // The nearest flower to the agent
    private Flower nearestFlower;

    [Tooltip("Speed to pitch up or down")] public float pitchSpeed = 100f;

    // The rigid body of the agent
    private new Rigidbody rigidbody;

    // Allows for smoother pitch changes
    private float smoothPitchChange;

    // Allows for smoother yaw changes
    private float smoothYawChange;

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
    ///     Called when an action is received from either the player input o
    ///     actions[i] represents:
    ///     Index 0: move vector x (+1 = right, -1 = left)
    ///     Index 1: move vector y (+1 = up, -1 = down)
    ///     Index 2: move vector z (+1 = forward, -1 = backward)
    ///     Index 3: pitch angle (+1 = pitch up, -1 = pitch down)
    ///     Index 4: yaw angle (+1 = turn right, -1 = turn left)
    /// </summary>
    /// <param name="actions">The actions to take</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Don't take actions if frozen
        if (frozen) return;

        // Calculate movement vector
        var move = new Vector3(actions.ContinuousActions.Array[0], actions.ContinuousActions.Array[1],
            actions.ContinuousActions.Array[2]);

        // Add force in the direction of the move vector
        rigidbody.AddForce(move * moveForce);

        // Get the current rotation
        var rotationVector = transform.rotation.eulerAngles;

        // Calculate pitch and yaw rotation
        var pitchChange = actions.ContinuousActions.Array[3];
        var yawChange = actions.ContinuousActions.Array[4];

        // Calculate smooth rotation changes
        smoothPitchChange = Mathf.MoveTowards(smoothPitchChange, pitchChange, 2f * Time.fixedDeltaTime);
        smoothYawChange = Mathf.MoveTowards(smoothYawChange, yawChange, 2f * Time.fixedDeltaTime);

        // Calculate new pitch and yaw based on smoothed values
        // Clamp pitch to avoid flipping upside down
        var pitch = rotationVector.x + smoothPitchChange * Time.fixedDeltaTime * pitchSpeed;
        if (pitch > 180f) pitch -= 360f;
        pitch = Mathf.Clamp(pitch, -MaxPitchAngle, MaxPitchAngle);

        var yaw = rotationVector.y + smoothYawChange * Time.fixedDeltaTime * yawSpeed;

        // Apply the new rotation
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
    }

    /// <summary>
    ///     Collect vector observations from the environment
    /// </summary>
    /// <param name="sensor">The vector sensor</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        // If nearestFlower is null, observe an empty array and return early
        if (nearestFlower == null)
        {
            sensor.AddObservation(new float[10]);
            return;
        }

        // Observe the agent's local rotation (4 observations)
        sensor.AddObservation(transform.localRotation.normalized);

        // Get a vector from the beak tip to the nearest flower
        var toFlower = nearestFlower.FlowerCenterPosition - beakTip.position;

        // Observe a normalized vector pointing to the nearest flower (3 observations)
        sensor.AddObservation(toFlower.normalized);

        // Observe a dot product that indicates whether the beak tip is in front of the flower (1 observation)
        // (+1 means that the beak tip is directly in front of the flower, -1 means directly behind)
        sensor.AddObservation(Vector3.Dot(toFlower.normalized, -nearestFlower.FlowerUpVector.normalized));

        // Observe a dot product that indicated whether the beak is pointing towards the flower (1 observation)
        // (+1 means that the beak is pointing directly at the flower, -1 means directly away)
        sensor.AddObservation(Vector3.Dot(beakTip.forward.normalized, -nearestFlower.FlowerUpVector.normalized));

        // Observe the relative distance from the beak tip to the flower (1 observation)
        sensor.AddObservation(toFlower.magnitude / FlowerArea.AreaDiameter);

        // 10 total observations
    }

    /// <summary>
    ///     When behaviour type is set to "Heuristic Only" on the agent's Behavior parameters
    ///     this function will be called. Its return values will be fed into
    ///     <see cref="OnActionReceived(ActionBuffers)" /> instead of using the neural network
    /// </summary>
    /// <param name="actionsOut">An output action array</param>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Create placeholders for all movement/turning
        var forward = Vector3.zero;
        var left = Vector3.zero;
        var up = Vector3.zero;
        var pitch = 0f;
        var yaw = 0f;

        // Convert keyboard inputs to movement and turning
        // All values should be between -1 and +1

        // Forward/backward
        if (Input.GetKey(KeyCode.W)) forward = transform.forward;
        else if (Input.GetKey(KeyCode.S)) forward = -transform.forward;

        // Left/right
        if (Input.GetKey(KeyCode.A)) left = transform.right;
        else if (Input.GetKey(KeyCode.D)) left = -transform.right;

        // Up.down
        if (Input.GetKey(KeyCode.E)) up = transform.up;
        else if (Input.GetKey(KeyCode.C)) up = -transform.up;

        // Pitch up/down
        if (Input.GetKey(KeyCode.UpArrow)) pitch = 1f;
        else if (Input.GetKey(KeyCode.DownArrow)) pitch = -1f;

        // Turn left/right
        if (Input.GetKey(KeyCode.LeftArrow)) yaw = 1f;
        else if (Input.GetKey(KeyCode.RightArrow)) yaw = -1f;

        // Combine the movement vector and normalize
        var combined = (forward + left + up).normalized;

        // Add the 3 movements values, pitch and yaw to the actionsOut array
        actionsOut.ContinuousActions.Array[0] = combined.x;
        actionsOut.ContinuousActions.Array[1] = combined.y;
        actionsOut.ContinuousActions.Array[2] = combined.z;
        actionsOut.ContinuousActions.Array[3] = pitch;
        actionsOut.ContinuousActions.Array[4] = yaw;
    }

    /// <summary>
    ///     Prevent the agent from moving and taking actions
    /// </summary>
    public void FreezeAgent()
    {
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supported in training");
        frozen = true;
        rigidbody.Sleep();
    }

    /// <summary>
    ///     Resume agent movement and actions
    /// </summary>
    public void UnfreezeAgent()
    {
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supported in training");
        frozen = false;
        rigidbody.WakeUp();
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
            safePositionFound = colliders.Length == 0;
        }

        Debug.Assert(safePositionFound, "Could not find a safe position to spawn");

        // Set the position and rotation
        transform.position = potentialPosition;
        transform.rotation = potentialRotation;
    }
}