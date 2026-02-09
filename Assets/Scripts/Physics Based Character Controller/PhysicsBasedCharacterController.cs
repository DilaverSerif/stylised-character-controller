using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// A floating-capsule oriented physics based character controller. Based on the approach devised by Toyful Games for Very Very Valet.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[DisallowMultipleComponent]
public class PhysicsBasedCharacterController : MonoBehaviour
{
    // Constants
    private const float GROUNDED_DISTANCE_TOLERANCE = 1.3f; // Allows for greater leniency as the value oscillates about the rideHeight
    private const float JUMP_RESET_TIME = 0.2f; // Time after landing before jump state resets
    private const float MIN_RIDE_HEIGHT = 0.1f;
    private const float MIN_RAY_LENGTH = 0.1f;
    private const float EPSILON_DISTANCE = 0.001f; // Small epsilon for distance comparisons
    private const float MIN_FIXED_DELTA_TIME = 0.0001f; // Minimum fixed delta time to prevent division by zero
    
    // Static readonly vectors to avoid allocations
    private static readonly Vector3 Vector3Down = Vector3.down;
    private static readonly Vector3 Vector3Up = Vector3.up;
    private static readonly Vector3 Vector3Zero = Vector3.zero;

    // Core components
    private Rigidbody _rb;
    private Vector3 _gravitationalForce;

    // Cached values for performance
    private Vector3 _cachedVelocity;
    private Vector3 _cachedAngularVelocity;
    private float _cachedFixedDeltaTime;
    private Vector3 _cachedPosition;
    private Quaternion _cachedRotation;
    private Transform _cachedTransform;
    private Transform _cachedParent;
    
    // Cached vectors to avoid allocations
    private Vector3 _cachedVelocityXZ;
    private Vector3 _cachedTargetPosition;

    // State variables
    private Vector3 _previousVelocity = Vector3Zero;
    private Vector3 _moveInput;
    private Vector3 _jumpInput;
    private Vector3 _goalVelocity = Vector3Zero;
    private float _speedFactor = 1f;
    private float _maxAccelForceFactor = 1f;
    private float _timeSinceJumpPressed = 0f;
    private float _timeSinceUngrounded = 0f;
    private float _timeSinceJump = 0f;
    private bool _shouldMaintainHeight = true;
    private bool _jumpReady = true;
    private bool _isJumping = false;
    private bool _prevGrounded = false;
    private bool _isGrounded = false;

    // Rotation state
    private enum lookDirectionOptions { velocity, acceleration, moveInput };
    private Quaternion _uprightTargetRot = Quaternion.identity;
    private Quaternion _lastTargetRot;
    private Vector3 _platformInitRot;
    private bool _wasLastRayHit;
    
    // Ground detection state
    private Vector3 _groundNormal = Vector3Up;
    private float _groundAngle = 0f;
    private float _cachedMass;

    [Header("Other:")]
    [Tooltip("Layer mask for terrain detection. Only objects on these layers will be considered as ground.")]
    [SerializeField] private LayerMask _terrainLayer;

    [Header("Height Spring:")]
    [Tooltip("Desired distance to ground (distance from the original raycast position, currently centre of transform).")]
    [Range(MIN_RIDE_HEIGHT, 10f)]
    [SerializeField] private float _rideHeight = 1.75f;
    [Tooltip("Max distance of raycast to ground (should be greater than the rideHeight).")]
    [Range(MIN_RAY_LENGTH, 20f)]
    [SerializeField] private float _rayToGroundLength = 3f;
    [Tooltip("Strength of spring force that maintains ride height.")]
    [Range(0f, 500f)]
    [SerializeField] private float _rideSpringStrength = 50f;
    
    /// <summary>
    /// Strength of spring force that maintains ride height.
    /// </summary>
    public float RideSpringStrength
    {
        get => _rideSpringStrength;
        set => _rideSpringStrength = Mathf.Clamp(value, 0f, 500f);
    }
    [Tooltip("Dampener of spring force that maintains ride height.")]
    [Range(0f, 50f)]
    [SerializeField] private float _rideSpringDamper = 5f;
    [Tooltip("Optional oscillator component for squash and stretch effects.")]
    [SerializeField] private Oscillator _squashAndStretchOcillator;

    [Header("Upright Spring:")]
    [Tooltip("Determines how the character's look direction is calculated: velocity, acceleration, or moveInput.")]
    [SerializeField] private lookDirectionOptions _characterLookDirection = lookDirectionOptions.velocity;
    [Tooltip("Strength of spring force that keeps character upright.")]
    [Range(0f, 500f)]
    [SerializeField] private float _uprightSpringStrength = 40f;
    [Tooltip("Dampener of spring force that keeps character upright.")]
    [Range(0f, 50f)]
    [SerializeField] private float _uprightSpringDamper = 5f;

    [Header("Movement:")]
    [Tooltip("Maximum speed the character can move.")]
    [Range(0f, 50f)]
    [SerializeField] private float _maxSpeed = 8f;
    [Tooltip("Acceleration rate for movement.")]
    [Range(0f, 1000f)]
    [SerializeField] private float _acceleration = 200f;
    [Tooltip("Maximum acceleration force that can be applied.")]
    [Range(0f, 1000f)]
    [SerializeField] private float _maxAccelForce = 150f;
    [Tooltip("Factor that determines how much the character leans when moving.")]
    [Range(0f, 1f)]
    [SerializeField] private float _leanFactor = 0.25f;
    [Tooltip("Animation curve that modifies acceleration based on velocity dot product.")]
    [SerializeField] private AnimationCurve _accelerationFactorFromDot;
    [Tooltip("Animation curve that modifies max acceleration force based on velocity dot product.")]
    [SerializeField] private AnimationCurve _maxAccelerationForceFactorFromDot;
    [Tooltip("Scale factor for movement forces on each axis (typically Y should be 0).")]
    [SerializeField] private Vector3 _moveForceScale = new Vector3(1f, 0f, 1f);

    [Header("Jump:")]
    [Tooltip("Multiplier for jump force.")]
    [Range(0f, 50f)]
    [SerializeField] private float _jumpForceFactor = 10f;
    [Tooltip("Gravity multiplier when rising (typically > 1).")]
    [Range(0f, 20f)]
    [SerializeField] private float _riseGravityFactor = 5f;
    [Tooltip("Gravity multiplier when falling (typically > 1, e.g. 5f).")]
    [Range(0f, 20f)]
    [SerializeField] private float _fallGravityFactor = 10f;
    [Tooltip("Gravity multiplier for low jumps when jump button is released early.")]
    [Range(0f, 20f)]
    [SerializeField] private float _lowJumpFactor = 2.5f;
    [Tooltip("Time window after pressing jump where jump can still be executed (shouldn't exceed jump time).")]
    [Range(0f, 1f)]
    [SerializeField] private float _jumpBuffer = 0.15f;
    [Tooltip("Time window after leaving ground where jump can still be executed.")]
    [Range(0f, 1f)]
    [SerializeField] private float _coyoteTime = 0.25f;

    [Header("Advanced Movement:")]
    [Tooltip("Multiplier for movement control while in the air (0 = no air control, 1 = full control).")]
    [Range(0f, 1f)]
    [SerializeField] private float _airControlMultiplier = 0.5f;
    [Tooltip("Maximum slope angle in degrees that the character can walk on (0-90).")]
    [Range(0f, 90f)]
    [SerializeField] private float _maxSlopeAngle = 45f;
    [Tooltip("Distance threshold for ground snapping when falling (0 = disabled).")]
    [Range(0f, 1f)]
    [SerializeField] private float _groundSnapDistance = 0.1f;

    [Header("Events:")]
    [Tooltip("Invoked when the character jumps.")]
    public UnityEvent OnJump;
    [Tooltip("Invoked when the character lands on the ground.")]
    public UnityEvent OnLand;
    [Tooltip("Invoked every frame while the character is grounded.")]
    public UnityEvent OnGrounded;
    [Tooltip("Invoked every frame while the character is not grounded.")]
    public UnityEvent OnUngrounded;
    [Tooltip("Invoked when the character starts moving.")]
    public UnityEvent OnMoveStarted;
    [Tooltip("Invoked when the character stops moving.")]
    public UnityEvent OnMoveStopped;

    // Public properties
    /// <summary>
    /// Whether the character is currently grounded.
    /// </summary>
    public bool IsGrounded => _isGrounded;

    /// <summary>
    /// Current linear velocity of the character.
    /// </summary>
    public Vector3 Velocity => _cachedVelocity;

    /// <summary>
    /// Current goal velocity for movement.
    /// </summary>
    public Vector3 GoalVelocity => _goalVelocity;

    /// <summary>
    /// Set the move input for the character.
    /// </summary>
    /// <param name="moveInput">The move input vector.</param>
    public void SetMoveInput(Vector3 moveInput)
    {
        bool wasMoving = _moveInput.sqrMagnitude > 0f;
        _moveInput = moveInput;
        bool isMoving = _moveInput.sqrMagnitude > 0f;

        if (!wasMoving && isMoving)
        {
            OnMoveStarted?.Invoke();
        }
        else if (wasMoving && !isMoving)
        {
            OnMoveStopped?.Invoke();
        }
    }

    /// <summary>
    /// Set the jump input for the character.
    /// </summary>
    /// <param name="jumpInput">The jump input vector (usually (0, 1, 0)).</param>
    /// <param name="jumpStarted">Whether the jump button was just pressed this frame.</param>
    public void SetJumpInput(Vector3 jumpInput, bool jumpStarted = false)
    {
        _jumpInput = jumpInput;

        if (jumpStarted)
        {
            _timeSinceJumpPressed = 0f;
        }
    }

    /// <summary>
    /// Set the speed factor multiplier for movement.
    /// </summary>
    /// <param name="factor">Speed multiplier (1.0 = normal speed).</param>
    public void SetSpeedFactor(float factor)
    {
        _speedFactor = Mathf.Max(0f, factor);
    }

    /// <summary>
    /// Set the maximum acceleration force factor multiplier.
    /// </summary>
    /// <param name="factor">Acceleration force multiplier (1.0 = normal acceleration).</param>
    public void SetMaxAccelForceFactor(float factor)
    {
        _maxAccelForceFactor = Mathf.Max(0f, factor);
    }

    /// <summary>
    /// Validates values in the editor and checks Rigidbody configuration.
    /// </summary>
    private void OnValidate()
    {
        _rideHeight = Mathf.Max(MIN_RIDE_HEIGHT, _rideHeight);
        _rayToGroundLength = Mathf.Max(MIN_RAY_LENGTH, _rayToGroundLength);
        
        if (_rayToGroundLength <= _rideHeight)
        {
            _rayToGroundLength = _rideHeight * 1.5f;
        }
        
        // Ensure AnimationCurves are initialized with default values if null
        if (_accelerationFactorFromDot == null)
        {
            _accelerationFactorFromDot = AnimationCurve.Linear(0f, 1f, 1f, 1f);
        }
        
        if (_maxAccelerationForceFactorFromDot == null)
        {
            _maxAccelerationForceFactorFromDot = AnimationCurve.Linear(0f, 1f, 1f, 1f);
        }
        
        // Validate Rigidbody configuration
        ValidateRigidbodyConfiguration();
    }
    
    /// <summary>
    /// Validates and optionally fixes Rigidbody configuration for optimal character controller behavior.
    /// </summary>
    private void ValidateRigidbodyConfiguration()
    {
        if (_rb == null)
        {
            _rb = GetComponent<Rigidbody>();
        }
        
        if (_rb == null) return;
        
        bool needsWarning = false;
        string warnings = "PhysicsBasedCharacterController: Rigidbody configuration issues detected:\n";
        
        // Check freezeRotation - X and Z should be frozen for upright character
        if (!_rb.freezeRotation)
        {
            needsWarning = true;
            warnings += "- freezeRotation should be enabled for stable character control\n";
        }
        
        // Check interpolation - Interpolate is recommended for smooth movement
        if (_rb.interpolation == RigidbodyInterpolation.None)
        {
            needsWarning = true;
            warnings += "- interpolation should be set to Interpolate for smooth movement\n";
        }
        
        // Check collision detection - Continuous recommended for fast movement
        if (_rb.collisionDetectionMode == CollisionDetectionMode.Discrete)
        {
            needsWarning = true;
            warnings += "- collisionDetectionMode should be Continuous or ContinuousDynamic for fast movement\n";
        }
        
        if (needsWarning)
        {
            warnings += "\nRecommended settings:\n";
            warnings += "- freezeRotation: true\n";
            warnings += "- interpolation: RigidbodyInterpolation.Interpolate\n";
            warnings += "- collisionDetectionMode: CollisionDetectionMode.Continuous\n";
            Debug.LogWarning(warnings, this);
        }
    }

    /// <summary>
    /// Prepare frequently used variables.
    /// </summary>
    private void Start()
    {
        _rb = GetComponent<Rigidbody>();
        if (_rb == null)
        {
            Debug.LogError("PhysicsBasedCharacterController: Rigidbody component not found!", this);
            enabled = false;
            return;
        }
        
        _cachedTransform = transform;
        _cachedMass = _rb.mass;
        _gravitationalForce = Physics.gravity * _cachedMass;
        
        // Validate Rigidbody configuration
        ValidateRigidbodyConfiguration();
        
        // Validate critical values
        if (_rideHeight <= 0f)
        {
            Debug.LogWarning($"PhysicsBasedCharacterController: _rideHeight must be positive. Using default value.", this);
            _rideHeight = 1.75f;
        }
        
        if (_rayToGroundLength <= _rideHeight)
        {
            Debug.LogWarning($"PhysicsBasedCharacterController: _rayToGroundLength should be greater than _rideHeight. Adjusting.", this);
            _rayToGroundLength = _rideHeight * 1.5f;
        }
        
        // Initialize AnimationCurves with default values if null
        if (_accelerationFactorFromDot == null)
        {
            _accelerationFactorFromDot = AnimationCurve.Linear(0f, 1f, 1f, 1f);
        }
        
        if (_maxAccelerationForceFactorFromDot == null)
        {
            _maxAccelerationForceFactorFromDot = AnimationCurve.Linear(0f, 1f, 1f, 1f);
        }
        
        // Initialize cached vectors
        _cachedVelocityXZ = Vector3Zero;
        _cachedTargetPosition = Vector3Zero;
    }

    /// <summary>
    /// Use the result of a Raycast to determine if the capsules distance from the ground is sufficiently close to the desired ride height such that the character can be considered 'grounded'.
    /// </summary>
    /// <param name="rayHitGround">Whether or not the Raycast hit anything.</param>
    /// <param name="rayHit">Information about the ray.</param>
    /// <returns>Whether or not the player is considered grounded.</returns>
    private bool CheckIfGrounded(bool rayHitGround, RaycastHit rayHit)
    {
        if (!rayHitGround) return false;
        
        // Check if slope is too steep
        if (_groundAngle > _maxSlopeAngle)
        {
            return false;
        }
        
        return rayHit.distance <= _rideHeight * GROUNDED_DISTANCE_TOLERANCE;
    }

    /// <summary>
    /// Gets the look desired direction for the character to look.
    /// The method for determining the look direction depends on the lookDirectionOption.
    /// </summary>
    /// <param name="lookDirectionOption">The factor which determines the look direction: velocity, acceleration or moveInput.</param>
    /// <returns>The desired look direction.</returns>
    private Vector3 GetLookDirection(lookDirectionOptions lookDirectionOption)
    {
        Vector3 lookDirection = Vector3Zero;
        
        if (lookDirectionOption == lookDirectionOptions.velocity || lookDirectionOption == lookDirectionOptions.acceleration)
        {
            Vector3 velocity = _cachedVelocity;
            velocity.y = 0f;
            
            if (lookDirectionOption == lookDirectionOptions.velocity)
            {
                lookDirection = velocity;
            }
            else if (lookDirectionOption == lookDirectionOptions.acceleration)
            {
                Vector3 deltaVelocity = velocity - _previousVelocity;
                _previousVelocity = velocity;
                
                // Prevent division by zero
                float deltaTime = Mathf.Max(_cachedFixedDeltaTime, MIN_FIXED_DELTA_TIME);
                Vector3 acceleration = deltaVelocity / deltaTime;
                lookDirection = acceleration;
            }
        }
        else if (lookDirectionOption == lookDirectionOptions.moveInput)
        {
            lookDirection = _moveInput;
        }
        
        return lookDirection;
    }

    /// <summary>
    /// Determines and triggers events, then calls the appropriate methods to move and float the character.
    /// </summary>
    private void FixedUpdate()
    {
        if (_rb == null) return;

        // Cache frequently accessed values
        _cachedFixedDeltaTime = Mathf.Max(Time.fixedDeltaTime, MIN_FIXED_DELTA_TIME);
        _cachedVelocity = _rb.linearVelocity;
        _cachedAngularVelocity = _rb.angularVelocity;
        _cachedPosition = _cachedTransform.position;
        _cachedRotation = _cachedTransform.rotation;
        _cachedParent = _cachedTransform.parent;

        // Perform ground detection
        RaycastHit rayHit;
        bool rayHitGround = RaycastToGround(out rayHit);
        SetPlatform(rayHit);

        // Update grounded state and trigger events
        _isGrounded = CheckIfGrounded(rayHitGround, rayHit);
        UpdateGroundedState();

        // Apply movement and jump forces
        CharacterMove(_moveInput, rayHit);
        CharacterJump(_jumpInput, _isGrounded, rayHit);

        // Maintain height and rotation
        if (rayHitGround && _shouldMaintainHeight)
        {
            MaintainHeight(rayHit);
            
            // Apply ground snapping if enabled and falling
            if (_groundSnapDistance > EPSILON_DISTANCE && _cachedVelocity.y < 0f)
            {
                ApplyGroundSnapping(rayHit);
            }
        }
        
        // Update gravitational force if mass changed
        if (Mathf.Abs(_rb.mass - _cachedMass) > EPSILON_DISTANCE)
        {
            _cachedMass = _rb.mass;
            _gravitationalForce = Physics.gravity * _cachedMass;
        }

        Vector3 lookDirection = GetLookDirection(_characterLookDirection);
        MaintainUpright(lookDirection, rayHit);

        _prevGrounded = _isGrounded;
    }

    /// <summary>
    /// Updates the grounded state and triggers appropriate events.
    /// </summary>
    private void UpdateGroundedState()
    {
        if (_isGrounded)
        {
            if (!_prevGrounded)
            {
                OnLand?.Invoke();
            }

            OnGrounded?.Invoke();
            _timeSinceUngrounded = 0f;

            if (_timeSinceJump > JUMP_RESET_TIME)
            {
                _isJumping = false;
            }
        }
        else
        {
            if (_prevGrounded)
            {
                OnUngrounded?.Invoke();
            }

            _timeSinceUngrounded += _cachedFixedDeltaTime;
        }
    }

    /// <summary>
    /// Perform raycast towards the ground.
    /// </summary>
    /// <param name="rayHit">Information about the raycast result.</param>
    /// <returns>Whether the ray hit the ground.</returns>
    private bool RaycastToGround(out RaycastHit rayHit)
    {
        Ray rayToGround = new Ray(_cachedPosition, Vector3Down);
        bool rayHitGround = Physics.Raycast(rayToGround, out rayHit, _rayToGroundLength, _terrainLayer.value);
        
        if (rayHitGround)
        {
            _groundNormal = rayHit.normal;
            _groundAngle = Vector3.Angle(_groundNormal, Vector3Up);
        }
        else
        {
            _groundNormal = Vector3Up;
            _groundAngle = 0f;
        }
        
        return rayHitGround;
    }

    /// <summary>
    /// Determines the relative velocity of the character to the ground beneath,
    /// Calculates and applies the oscillator force to bring the character towards the desired ride height.
    /// Additionally applies the oscillator force to the squash and stretch oscillator, and any object beneath.
    /// </summary>
    /// <param name="rayHit">Information about the RaycastToGround.</param>
    /// <summary>
    /// Applies ground snapping when falling near the ground.
    /// </summary>
    /// <param name="rayHit">Information about the RaycastToGround.</param>
    private void ApplyGroundSnapping(RaycastHit rayHit)
    {
        if (_rb == null) return;
        
        float distanceToGround = rayHit.distance;
        float snapThreshold = _rideHeight + _groundSnapDistance;
        
        // Only snap when falling and close to ground
        if (distanceToGround > _rideHeight && distanceToGround < snapThreshold)
        {
            float snapAmount = distanceToGround - _rideHeight;
            _cachedTargetPosition = _cachedPosition;
            _cachedTargetPosition.y -= snapAmount;
            _rb.MovePosition(_cachedTargetPosition);
        }
    }

    /// <summary>
    /// Determines the relative velocity of the character to the ground beneath,
    /// Calculates and applies the oscillator force to bring the character towards the desired ride height.
    /// Additionally applies the oscillator force to the squash and stretch oscillator, and any object beneath.
    /// </summary>
    /// <param name="rayHit">Information about the RaycastToGround.</param>
    private void MaintainHeight(RaycastHit rayHit)
    {
        if (_rb == null) return;

        Vector3 vel = _cachedVelocity;
        Vector3 otherVel = Vector3Zero;
        Rigidbody hitBody = rayHit.rigidbody;
        
        if (hitBody != null)
        {
            otherVel = hitBody.linearVelocity;
        }
        
        float rayDirVel = Vector3.Dot(Vector3Down, vel);
        float otherDirVel = Vector3.Dot(Vector3Down, otherVel);
        float relVel = rayDirVel - otherDirVel;
        float currHeight = rayHit.distance - _rideHeight;
        float springForce = (currHeight * _rideSpringStrength) - (relVel * _rideSpringDamper);
        Vector3 maintainHeightForce = -_gravitationalForce + springForce * Vector3Down;
        Vector3 oscillationForce = springForce * Vector3Down;
        
        _rb.AddForce(maintainHeightForce);

        // Apply force to squash and stretch oscillator if available
        if (_squashAndStretchOcillator != null)
        {
            _squashAndStretchOcillator.ApplyForce(oscillationForce);
        }

        // Apply force to objects beneath
        if (hitBody != null)
        {
            hitBody.AddForceAtPosition(-maintainHeightForce, rayHit.point);
        }
    }

    /// <summary>
    /// Determines the desired y rotation for the character, with account for platform rotation.
    /// </summary>
    /// <param name="yLookAt">The input look rotation.</param>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void CalculateTargetRotation(Vector3 yLookAt, RaycastHit rayHit)
    {
        bool isOnRigidPlatform = rayHit.rigidbody != null;
        
        if (_wasLastRayHit && !isOnRigidPlatform)
        {
            _lastTargetRot = _uprightTargetRot;
            UpdatePlatformInitRotation();
        }
        
        _wasLastRayHit = isOnRigidPlatform;

        if (yLookAt.sqrMagnitude > EPSILON_DISTANCE)
        {
            _uprightTargetRot = Quaternion.LookRotation(yLookAt, Vector3Up);
            _lastTargetRot = _uprightTargetRot;
            UpdatePlatformInitRotation();
        }
        else
        {
            if (_cachedParent != null)
            {
                Vector3 platformRot = _cachedParent.rotation.eulerAngles;
                Vector3 deltaPlatformRot = platformRot - _platformInitRot;
                float yAngle = _lastTargetRot.eulerAngles.y + deltaPlatformRot.y;
                _uprightTargetRot = Quaternion.Euler(0f, yAngle, 0f);
            }
        }
    }

    /// <summary>
    /// Updates the initial platform rotation reference.
    /// </summary>
    private void UpdatePlatformInitRotation()
    {
        if (_cachedParent != null)
        {
            _platformInitRot = _cachedParent.rotation.eulerAngles;
        }
        else
        {
            _platformInitRot = Vector3Zero;
        }
    }

    /// <summary>
    /// Adds torque to the character to keep the character upright, acting as a torsional oscillator (i.e. vertically flipped pendulum).
    /// </summary>
    /// <param name="yLookAt">The input look rotation.</param>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void MaintainUpright(Vector3 yLookAt, RaycastHit rayHit)
    {
        if (_rb == null) return;

        CalculateTargetRotation(yLookAt, rayHit);

        Quaternion currentRot = _cachedRotation;
        Quaternion toGoal = MathsUtils.ShortestRotation(_uprightTargetRot, currentRot);

        Vector3 rotAxis;
        float rotDegrees;

        toGoal.ToAngleAxis(out rotDegrees, out rotAxis);
        rotAxis.Normalize();

        float rotRadians = rotDegrees * Mathf.Deg2Rad;

        _rb.AddTorque((rotAxis * (rotRadians * _uprightSpringStrength)) - (_cachedAngularVelocity * _uprightSpringDamper));
    }

    /// <summary>
    /// Set the transform parent to be the result of RaycastToGround.
    /// If the raycast didn't hit, then unset the transform parent.
    /// </summary>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void SetPlatform(RaycastHit rayHit)
    {
        if (rayHit.transform != null)
        {
            RigidPlatform rigidPlatform = rayHit.transform.GetComponent<RigidPlatform>();
            if (rigidPlatform != null && rigidPlatform.rigidParent != null)
            {
                _cachedTransform.SetParent(rigidPlatform.rigidParent.transform);
                return;
            }
        }
        _cachedTransform.SetParent(null);
    }

    /// <summary>
    /// Evaluates an AnimationCurve safely, returning 1.0 if the curve is null.
    /// </summary>
    /// <param name="curve">The AnimationCurve to evaluate.</param>
    /// <param name="time">The time at which to evaluate the curve.</param>
    /// <returns>The evaluated value, or 1.0 if the curve is null.</returns>
    private float EvaluateCurveSafe(AnimationCurve curve, float time)
    {
        return curve != null ? curve.Evaluate(time) : 1f;
    }

    /// <summary>
    /// Calculates the goal velocity based on input and current state, with air control and slope consideration.
    /// </summary>
    /// <param name="moveInput">The player movement input.</param>
    /// <param name="isGrounded">Whether the character is grounded.</param>
    /// <returns>The target goal velocity.</returns>
    private Vector3 CalculateGoalVelocity(Vector3 moveInput, bool isGrounded)
    {
        Vector3 unitGoal = moveInput;
        float goalMagnitude = unitGoal.magnitude;
        Vector3 goalVel = Vector3Zero;

        if (goalMagnitude >= EPSILON_DISTANCE)
        {
            unitGoal /= goalMagnitude; // Normalize
            
            // Apply air control multiplier if not grounded
            float speedMultiplier = _speedFactor;
            if (!isGrounded)
            {
                speedMultiplier *= _airControlMultiplier;
            }
            
            // Project movement direction onto ground plane if on slope
            if (isGrounded && _groundAngle > EPSILON_DISTANCE && _groundAngle < _maxSlopeAngle)
            {
                Vector3 projectedDirection = Vector3.ProjectOnPlane(unitGoal, _groundNormal).normalized;
                if (projectedDirection.sqrMagnitude > EPSILON_DISTANCE)
                {
                    unitGoal = projectedDirection;
                }
            }
            
            goalVel = unitGoal * _maxSpeed * speedMultiplier;
            
            float currentGoalVelMagnitude = _goalVelocity.magnitude;
            Vector3 currentUnitVel = currentGoalVelMagnitude > EPSILON_DISTANCE ? _goalVelocity / currentGoalVelMagnitude : Vector3Zero;
            
            float inputVelDot = Vector3.Dot(unitGoal, currentUnitVel);
            float inputAccel = _acceleration * EvaluateCurveSafe(_accelerationFactorFromDot, inputVelDot);
            
            _goalVelocity = Vector3.MoveTowards(_goalVelocity, goalVel, inputAccel * _cachedFixedDeltaTime);
        }
        else
        {
            // No input, gradually reduce goal velocity
            _goalVelocity = Vector3.MoveTowards(_goalVelocity, Vector3Zero, _acceleration * _cachedFixedDeltaTime);
        }

        return goalVel;
    }

    /// <summary>
    /// Applies movement forces to the character.
    /// </summary>
    /// <param name="moveInput">The player movement input.</param>
    private void ApplyMovementForces(Vector3 moveInput)
    {
        Vector3 unitGoal = moveInput;
        float goalMagnitude = unitGoal.magnitude;
        
        // Normalize if needed
        if (goalMagnitude >= EPSILON_DISTANCE)
        {
            unitGoal /= goalMagnitude;
        }

        // Always calculate neededAccel to apply forces (including braking when stopping)
        float goalVelMagnitude = _goalVelocity.magnitude;
        Vector3 unitVel = goalVelMagnitude > EPSILON_DISTANCE ? _goalVelocity / goalVelMagnitude : Vector3Zero;
        float velDot = goalMagnitude >= EPSILON_DISTANCE ? Vector3.Dot(unitGoal, unitVel) : -1f; // Negative when stopping
        float accel = _acceleration * EvaluateCurveSafe(_accelerationFactorFromDot, velDot);
        float maxAccel = _maxAccelForce * EvaluateCurveSafe(_maxAccelerationForceFactorFromDot, velDot) * _maxAccelForceFactor;

        Vector3 neededAccel = (_goalVelocity - _cachedVelocity) / _cachedFixedDeltaTime;
        neededAccel = Vector3.ClampMagnitude(neededAccel, maxAccel);

        // Using AddForceAtPosition to both move the player and cause the player to lean in the direction of input
        float leanOffset = _cachedTransform.localScale.y * _leanFactor;
        Vector3 forcePosition = _cachedPosition;
        forcePosition.y += leanOffset;
        _rb.AddForceAtPosition(Vector3.Scale(neededAccel * _rb.mass, _moveForceScale), forcePosition);
    }

    /// <summary>
    /// Apply forces to move the character up to a maximum acceleration, with consideration to acceleration graphs.
    /// </summary>
    /// <param name="moveInput">The player movement input.</param>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void CharacterMove(Vector3 moveInput, RaycastHit rayHit)
    {
        if (_rb == null) return;

        CalculateGoalVelocity(moveInput, _isGrounded);
        ApplyMovementForces(moveInput);
    }

    /// <summary>
    /// Apply force to cause the character to perform a single jump, including coyote time and a jump input buffer.
    /// Jump mechanics: Resets vertical velocity to ensure consistent jump height regardless of initial velocity.
    /// Adjusts position to ride height if raycast hit to maintain proper jump start position.
    /// </summary>
    /// <param name="jumpInput">The player jump input.</param>
    /// <param name="grounded">Whether or not the player is considered grounded.</param>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void CharacterJump(Vector3 jumpInput, bool grounded, RaycastHit rayHit)
    {
        if (_rb == null) return;

        _timeSinceJumpPressed += _cachedFixedDeltaTime;
        _timeSinceJump += _cachedFixedDeltaTime;
        
        // Apply gravity modifiers based on vertical velocity
        ApplyGravityModifiers(grounded, jumpInput);
        
        // Check if jump should be executed (jump buffer + coyote time)
        if (ShouldExecuteJump())
        {
            ExecuteJump(rayHit);
        }
    }

    /// <summary>
    /// Applies gravity modifiers based on vertical velocity and jump state.
    /// </summary>
    /// <param name="grounded">Whether the character is grounded.</param>
    /// <param name="jumpInput">The current jump input.</param>
    private void ApplyGravityModifiers(bool grounded, Vector3 jumpInput)
    {
        float verticalVelocity = _cachedVelocity.y;
        
        if (verticalVelocity < 0f)
        {
            _shouldMaintainHeight = true;
            _jumpReady = true;
            if (!grounded)
            {
                // Increase downforce for a sudden plummet.
                _rb.AddForce(_gravitationalForce * (_fallGravityFactor - 1f));
            }
        }
        else if (verticalVelocity > 0f)
        {
            if (!grounded)
            {
                if (_isJumping)
                {
                    _rb.AddForce(_gravitationalForce * (_riseGravityFactor - 1f));
                }
                if (jumpInput.sqrMagnitude < EPSILON_DISTANCE)
                {
                    // Impede the jump height to achieve a low jump.
                    _rb.AddForce(_gravitationalForce * (_lowJumpFactor - 1f));
                }
            }
        }
    }

    /// <summary>
    /// Determines if a jump should be executed based on jump buffer and coyote time.
    /// </summary>
    /// <returns>True if jump should be executed.</returns>
    private bool ShouldExecuteJump()
    {
        return _timeSinceJumpPressed < _jumpBuffer && 
               _timeSinceUngrounded < _coyoteTime && 
               _jumpReady;
    }

    /// <summary>
    /// Executes the jump by applying forces and updating state.
    /// </summary>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void ExecuteJump(RaycastHit rayHit)
    {
        _jumpReady = false;
        _shouldMaintainHeight = false;
        _isJumping = true;
        
        // Reset vertical velocity to ensure consistent jump height
        Vector3 currentVel = _rb.linearVelocity;
        _cachedVelocityXZ.x = currentVel.x;
        _cachedVelocityXZ.y = 0f;
        _cachedVelocityXZ.z = currentVel.z;
        _rb.linearVelocity = _cachedVelocityXZ;
        
        // Adjust position to ride height if raycast hit to maintain proper jump start position
        // Use MovePosition instead of direct position assignment to respect physics simulation
        if (rayHit.distance > EPSILON_DISTANCE)
        {
            Vector3 currentPos = _rb.position;
            float heightAdjustment = rayHit.distance - _rideHeight;
            _cachedTargetPosition = currentPos;
            _cachedTargetPosition.y -= heightAdjustment;
            _rb.MovePosition(_cachedTargetPosition);
        }
        
        _rb.AddForce(Vector3Up * _jumpForceFactor, ForceMode.Impulse);
        _timeSinceJumpPressed = _jumpBuffer; // Prevent further jumps if player lands before jump timer exceeds buffer
        _timeSinceJump = 0f;

        OnJump?.Invoke();
    }

}
