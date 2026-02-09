using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// A floating-capsule oriented physics based character controller. Based on the approach devised by Toyful Games for Very Very Valet.
/// </summary>
public class PhysicsBasedCharacterController : MonoBehaviour
{
    // Constants
    private const float GROUNDED_DISTANCE_TOLERANCE = 1.3f; // Allows for greater leniency as the value oscillates about the rideHeight
    private const float JUMP_RESET_TIME = 0.2f; // Time after landing before jump state resets

    // Core components
    private Rigidbody _rb;
    private Vector3 _gravitationalForce;
    private Vector3 _rayDir = Vector3.down;

    // State variables
    private Vector3 _previousVelocity = Vector3.zero;
    private Vector3 _moveInput;
    private Vector3 _jumpInput;
    private Vector3 _goalVelocity = Vector3.zero;
    private float _speedFactor = 1f;
    private float _maxAccelForceFactor = 1f;
    private float _timeSinceJumpPressed = 0f;
    private float _timeSinceUngrounded = 0f;
    private float _timeSinceJump = 0f;
    private bool _shouldMaintainHeight = true;
    private bool _jumpReady = true;
    private bool _isJumping = false;
    private bool _prevGrounded = false;

    // Rotation state
    private enum lookDirectionOptions { velocity, acceleration, moveInput };
    private Quaternion _uprightTargetRot = Quaternion.identity;
    private Quaternion _lastTargetRot;
    private Vector3 _platformInitRot;
    private bool didLastRayHit;

    [Header("Other:")]
    [SerializeField] private LayerMask _terrainLayer;

    [Header("Height Spring:")]
    [SerializeField] private float _rideHeight = 1.75f; // rideHeight: desired distance to ground (Note, this is distance from the original raycast position (currently centre of transform)). 
    [SerializeField] private float _rayToGroundLength = 3f; // rayToGroundLength: max distance of raycast to ground (Note, this should be greater than the rideHeight).
    [SerializeField] public float _rideSpringStrength = 50f; // rideSpringStrength: strength of spring. (?)
    [SerializeField] private float _rideSpringDamper = 5f; // rideSpringDampener: dampener of spring. (?)
    [SerializeField] private Oscillator _squashAndStretchOcillator;

    [Header("Upright Spring:")]
    [SerializeField] private lookDirectionOptions _characterLookDirection = lookDirectionOptions.velocity;
    [SerializeField] private float _uprightSpringStrength = 40f;
    [SerializeField] private float _uprightSpringDamper = 5f;

    [Header("Movement:")]
    [SerializeField] private float _maxSpeed = 8f;
    [SerializeField] private float _acceleration = 200f;
    [SerializeField] private float _maxAccelForce = 150f;
    [SerializeField] private float _leanFactor = 0.25f;
    [SerializeField] private AnimationCurve _accelerationFactorFromDot;
    [SerializeField] private AnimationCurve _maxAccelerationForceFactorFromDot;
    [SerializeField] private Vector3 _moveForceScale = new Vector3(1f, 0f, 1f);

    [Header("Jump:")]
    [SerializeField] private float _jumpForceFactor = 10f;
    [SerializeField] private float _riseGravityFactor = 5f;
    [SerializeField] private float _fallGravityFactor = 10f; // typically > 1f (i.e. 5f).
    [SerializeField] private float _lowJumpFactor = 2.5f;
    [SerializeField] private float _jumpBuffer = 0.15f; // Note, jumpBuffer shouldn't really exceed the time of the jump.
    [SerializeField] private float _coyoteTime = 0.25f;

    [Header("Events:")]
    public UnityEvent OnJump;
    public UnityEvent OnLand;
    public UnityEvent OnGrounded;
    public UnityEvent OnUngrounded;
    public UnityEvent OnMoveStarted;
    public UnityEvent OnMoveStopped;

    /// <summary>
    /// Set the move input for the character.
    /// </summary>
    /// <param name="moveInput">The move input vector.</param>
    public void SetMoveInput(Vector3 moveInput)
    {
        bool wasMoving = _moveInput.magnitude != 0;
        _moveInput = moveInput;
        bool isMoving = _moveInput.magnitude != 0;

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
    /// Prepare frequently used variables.
    /// </summary>
    private void Start()
    {
        _rb = GetComponent<Rigidbody>();
        if (_rb == null)
        {
            Debug.LogError("PhysicsBasedCharacterController: Rigidbody component not found!", this);
            return;
        }
        _gravitationalForce = Physics.gravity * _rb.mass;
    }

    /// <summary>
    /// Use the result of a Raycast to determine if the capsules distance from the ground is sufficiently close to the desired ride height such that the character can be considered 'grounded'.
    /// </summary>
    /// <param name="rayHitGround">Whether or not the Raycast hit anything.</param>
    /// <param name="rayHit">Information about the ray.</param>
    /// <returns>Whether or not the player is considered grounded.</returns>
    private bool CheckIfGrounded(bool rayHitGround, RaycastHit rayHit)
    {
        return rayHitGround && rayHit.distance <= _rideHeight * GROUNDED_DISTANCE_TOLERANCE;
    }

    /// <summary>
    /// Gets the look desired direction for the character to look.
    /// The method for determining the look direction is depends on the lookDirectionOption.
    /// </summary>
    /// <param name="lookDirectionOption">The factor which determines the look direction: velocity, acceleration or moveInput.</param>
    /// <returns>The desired look direction.</returns>
    private Vector3 GetLookDirection(lookDirectionOptions lookDirectionOption)
    {
        Vector3 lookDirection = Vector3.zero;
        if (lookDirectionOption == lookDirectionOptions.velocity || lookDirectionOption == lookDirectionOptions.acceleration)
        {
            Vector3 velocity = _rb.linearVelocity;
            velocity.y = 0f;
            if (lookDirectionOption == lookDirectionOptions.velocity)
            {
                lookDirection = velocity;
            }
            else if (lookDirectionOption == lookDirectionOptions.acceleration)
            {
                Vector3 deltaVelocity = velocity - _previousVelocity;
                _previousVelocity = velocity;
                Vector3 acceleration = deltaVelocity / Time.fixedDeltaTime;
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
        (bool rayHitGround, RaycastHit rayHit) = RaycastToGround();
        SetPlatform(rayHit);

        bool grounded = CheckIfGrounded(rayHitGround, rayHit);
        if (grounded)
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

            _timeSinceUngrounded += Time.fixedDeltaTime;
        }

        CharacterMove(_moveInput, rayHit);
        CharacterJump(_jumpInput, grounded, rayHit);

        if (rayHitGround && _shouldMaintainHeight)
        {
            MaintainHeight(rayHit);
        }

        Vector3 lookDirection = GetLookDirection(_characterLookDirection);
        MaintainUpright(lookDirection, rayHit);

        _prevGrounded = grounded;
    }

    /// <summary>
    /// Perfom raycast towards the ground.
    /// </summary>
    /// <returns>Whether the ray hit the ground, and information about the ray.</returns>
    private (bool, RaycastHit) RaycastToGround()
    {
        RaycastHit rayHit;
        Ray rayToGround = new Ray(transform.position, _rayDir);
        bool rayHitGround = Physics.Raycast(rayToGround, out rayHit, _rayToGroundLength, _terrainLayer.value);
        return (rayHitGround, rayHit);
    }

    /// <summary>
    /// Determines the relative velocity of the character to the ground beneath,
    /// Calculates and applies the oscillator force to bring the character towards the desired ride height.
    /// Additionally applies the oscillator force to the squash and stretch oscillator, and any object beneath.
    /// </summary>
    /// <param name="rayHit">Information about the RaycastToGround.</param>
    private void MaintainHeight(RaycastHit rayHit)
    {
        Vector3 vel = _rb.linearVelocity;
        Vector3 otherVel = Vector3.zero;
        Rigidbody hitBody = rayHit.rigidbody;
        if (hitBody != null)
        {
            otherVel = hitBody.linearVelocity;
        }
        float rayDirVel = Vector3.Dot(_rayDir, vel);
        float otherDirVel = Vector3.Dot(_rayDir, otherVel);

        float relVel = rayDirVel - otherDirVel;
        float currHeight = rayHit.distance - _rideHeight;
        float springForce = (currHeight * _rideSpringStrength) - (relVel * _rideSpringDamper);
        Vector3 maintainHeightForce = - _gravitationalForce + springForce * Vector3.down;
        Vector3 oscillationForce = springForce * Vector3.down;
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
    private void CalculateTargetRotation(Vector3 yLookAt, RaycastHit rayHit = new RaycastHit())
    {
        if (didLastRayHit)
        {
            _lastTargetRot = _uprightTargetRot;
            if (transform.parent != null)
            {
                _platformInitRot = transform.parent.rotation.eulerAngles;
            }
            else
            {
                _platformInitRot = Vector3.zero;
            }
        }
        
        didLastRayHit = (rayHit.rigidbody == null);

        if (yLookAt != Vector3.zero)
        {
            _uprightTargetRot = Quaternion.LookRotation(yLookAt, Vector3.up);
            _lastTargetRot = _uprightTargetRot;
            if (transform.parent != null)
            {
                _platformInitRot = transform.parent.rotation.eulerAngles;
            }
            else
            {
                _platformInitRot = Vector3.zero;
            }
        }
        else
        {
            if (transform.parent != null)
            {
                Vector3 platformRot = transform.parent.rotation.eulerAngles;
                Vector3 deltaPlatformRot = platformRot - _platformInitRot;
                float yAngle = _lastTargetRot.eulerAngles.y + deltaPlatformRot.y;
                _uprightTargetRot = Quaternion.Euler(new Vector3(0f, yAngle, 0f));
            }
        }
    }

    /// <summary>
    /// Adds torque to the character to keep the character upright, acting as a torsional oscillator (i.e. vertically flipped pendulum).
    /// </summary>
    /// <param name="yLookAt">The input look rotation.</param>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void MaintainUpright(Vector3 yLookAt, RaycastHit rayHit = new RaycastHit())
    {
        CalculateTargetRotation(yLookAt, rayHit);

        Quaternion currentRot = transform.rotation;
        Quaternion toGoal = MathsUtils.ShortestRotation(_uprightTargetRot, currentRot);

        Vector3 rotAxis;
        float rotDegrees;

        toGoal.ToAngleAxis(out rotDegrees, out rotAxis);
        rotAxis.Normalize();

        float rotRadians = rotDegrees * Mathf.Deg2Rad;

        _rb.AddTorque((rotAxis * (rotRadians * _uprightSpringStrength)) - (_rb.angularVelocity * _uprightSpringDamper));
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
                transform.SetParent(rigidPlatform.rigidParent.transform);
                return;
            }
        }
        transform.SetParent(null);
    }

    /// <summary>
    /// Apply forces to move the character up to a maximum acceleration, with consideration to acceleration graphs.
    /// </summary>
    /// <param name="moveInput">The player movement input.</param>
    /// <param name="rayHit">The rayHit towards the platform.</param>
    private void CharacterMove(Vector3 moveInput, RaycastHit rayHit)
    {
        Vector3 unitGoal = moveInput;
        Vector3 unitVel = _goalVelocity.normalized;
        float velDot = Vector3.Dot(unitGoal, unitVel);
        float accel = _acceleration * _accelerationFactorFromDot.Evaluate(velDot);
        Vector3 goalVel = unitGoal * _maxSpeed * _speedFactor;
        
        _goalVelocity = Vector3.MoveTowards(_goalVelocity,
                                        goalVel,
                                        accel * Time.fixedDeltaTime);
        Vector3 neededAccel = (_goalVelocity - _rb.linearVelocity) / Time.fixedDeltaTime;
        float maxAccel = _maxAccelForce * _maxAccelerationForceFactorFromDot.Evaluate(velDot) * _maxAccelForceFactor;
        neededAccel = Vector3.ClampMagnitude(neededAccel, maxAccel);
        // Using AddForceAtPosition to both move the player and cause the player to lean in the direction of input
        _rb.AddForceAtPosition(Vector3.Scale(neededAccel * _rb.mass, _moveForceScale), transform.position + new Vector3(0f, transform.localScale.y * _leanFactor, 0f));
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
        _timeSinceJumpPressed += Time.fixedDeltaTime;
        _timeSinceJump += Time.fixedDeltaTime;
        
        if (_rb.linearVelocity.y < 0)
        {
            _shouldMaintainHeight = true;
            _jumpReady = true;
            if (!grounded)
            {
                // Increase downforce for a sudden plummet.
                _rb.AddForce(_gravitationalForce * (_fallGravityFactor - 1f));
            }
        }
        else if (_rb.linearVelocity.y > 0)
        {
            if (!grounded)
            {
                if (_isJumping)
                {
                    _rb.AddForce(_gravitationalForce * (_riseGravityFactor - 1f));
                }
                if (jumpInput == Vector3.zero)
                {
                    // Impede the jump height to achieve a low jump.
                    _rb.AddForce(_gravitationalForce * (_lowJumpFactor - 1f));
                }
            }
        }

        if (_timeSinceJumpPressed < _jumpBuffer)
        {
            if (_timeSinceUngrounded < _coyoteTime)
            {
                if (_jumpReady)
                {
                    _jumpReady = false;
                    _shouldMaintainHeight = false;
                    _isJumping = true;
                    
                    // Reset vertical velocity to ensure consistent jump height
                    _rb.linearVelocity = new Vector3(_rb.linearVelocity.x, 0f, _rb.linearVelocity.z);
                    
                    // Adjust position to ride height if raycast hit to maintain proper jump start position
                    if (rayHit.distance != 0)
                    {
                        _rb.position = new Vector3(_rb.position.x, _rb.position.y - (rayHit.distance - _rideHeight), _rb.position.z);
                    }
                    
                    _rb.AddForce(Vector3.up * _jumpForceFactor, ForceMode.Impulse);
                    _timeSinceJumpPressed = _jumpBuffer; // Prevent further jumps if player lands before jump timer exceeds buffer
                    _timeSinceJump = 0f;

                    OnJump?.Invoke();
                }
            }
        }
    }
}
