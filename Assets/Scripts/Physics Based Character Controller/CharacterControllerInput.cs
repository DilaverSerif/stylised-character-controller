using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(PhysicsBasedCharacterController))]
public class CharacterControllerInput : MonoBehaviour
{
    [SerializeField] private PhysicsBasedCharacterController _controller;
    [SerializeField] private bool _adjustInputsToCameraAngle = true;

    private Vector2 _moveContext;

    private void Awake()
    {
        if (_controller == null)
        {
            _controller = GetComponent<PhysicsBasedCharacterController>();
        }
    }

    // Support for "Send Messages" behavior
    public void OnMove(InputValue value)
    {
        _moveContext = value.Get<Vector2>();
    }

    public void OnJump(InputValue value)
    {
        bool isPressed = value.isPressed;
        _controller.SetJumpInput(new Vector3(0, isPressed ? 1f : 0f, 0), isPressed);
    }

    /// <summary>
    /// Reads the player movement input.
    /// </summary>
    /// <param name="context">The move input's context.</param>
    public void MoveInputAction(InputAction.CallbackContext context)
    {
        _moveContext = context.ReadValue<Vector2>();
    }

    /// <summary>
    /// Reads the player jump input.
    /// </summary>
    /// <param name="context">The jump input's context.</param>
    public void JumpInputAction(InputAction.CallbackContext context)
    {
        float jumpValue = context.ReadValue<float>();
        _controller.SetJumpInput(new Vector3(0, jumpValue, 0), context.started);
    }

    private void Update()
    {
        Vector3 moveInput = new Vector3(_moveContext.x, 0, _moveContext.y);

        if (_adjustInputsToCameraAngle)
        {
            moveInput = AdjustInputToFaceCamera(moveInput);
        }

        _controller.SetMoveInput(moveInput);
    }

    /// <summary>
    /// Adjusts the input, so that the movement matches input regardless of camera rotation.
    /// </summary>
    /// <param name="moveInput">The player movement input.</param>
    /// <returns>The camera corrected movement input.</returns>
    private Vector3 AdjustInputToFaceCamera(Vector3 moveInput)
    {
        if (Camera.main == null) return moveInput;
        float facing = Camera.main.transform.eulerAngles.y;
        return (Quaternion.Euler(0, facing, 0) * moveInput);
    }
}
