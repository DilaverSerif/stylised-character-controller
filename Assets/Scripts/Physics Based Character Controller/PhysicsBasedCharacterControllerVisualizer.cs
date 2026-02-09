#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Reflection;

/// <summary>
/// MonoBehaviour component for visualizing PhysicsBasedCharacterController using Unity Handles.
/// Only works in Unity Editor.
/// </summary>
[ExecuteInEditMode]
[RequireComponent(typeof(PhysicsBasedCharacterController))]
public class PhysicsBasedCharacterControllerVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    [Tooltip("Enable scene visualization")]
    [SerializeField] private bool showVisualization = false;
    
    [Tooltip("Show basic information labels")]
    [SerializeField] private bool showBasicInfo = true;
    
    [Tooltip("Show advanced information labels")]
    [SerializeField] private bool showAdvancedInfo = false;
    
    private PhysicsBasedCharacterController controller;
    private SerializedObject serializedController;
    
    // SerializedProperty references
    private SerializedProperty rideHeightProp;
    private SerializedProperty rayToGroundLengthProp;
    private SerializedProperty groundSnapDistanceProp;
    private SerializedProperty rideSpringStrengthProp;
    private SerializedProperty rideSpringDamperProp;
    private SerializedProperty uprightSpringStrengthProp;
    private SerializedProperty uprightSpringDamperProp;
    
    // Reflection fields for private runtime data
    private FieldInfo groundNormalField;
    private FieldInfo groundAngleField;
    private FieldInfo cachedTransformField;
    
    private const float GROUNDED_DISTANCE_TOLERANCE = 1.3f;
    private const float EPSILON_DISTANCE = 0.001f;
    
    private void OnEnable()
    {
        // Get controller reference
        controller = GetComponent<PhysicsBasedCharacterController>();
        if (controller == null)
        {
            Debug.LogWarning("PhysicsBasedCharacterControllerVisualizer: PhysicsBasedCharacterController component not found!", this);
            enabled = false;
            return;
        }
        
        // Create SerializedObject for accessing serialized properties
        serializedController = new SerializedObject(controller);
        
        // Cache SerializedProperties
        rideHeightProp = serializedController.FindProperty("_rideHeight");
        rayToGroundLengthProp = serializedController.FindProperty("_rayToGroundLength");
        groundSnapDistanceProp = serializedController.FindProperty("_groundSnapDistance");
        rideSpringStrengthProp = serializedController.FindProperty("_rideSpringStrength");
        rideSpringDamperProp = serializedController.FindProperty("_rideSpringDamper");
        uprightSpringStrengthProp = serializedController.FindProperty("_uprightSpringStrength");
        uprightSpringDamperProp = serializedController.FindProperty("_uprightSpringDamper");
        
        // Cache reflection fields for runtime data
        System.Type controllerType = typeof(PhysicsBasedCharacterController);
        groundNormalField = controllerType.GetField("_groundNormal", BindingFlags.NonPublic | BindingFlags.Instance);
        groundAngleField = controllerType.GetField("_groundAngle", BindingFlags.NonPublic | BindingFlags.Instance);
        cachedTransformField = controllerType.GetField("_cachedTransform", BindingFlags.NonPublic | BindingFlags.Instance);
        
        // Register SceneView callback
        SceneView.duringSceneGui += OnSceneGUI;
    }
    
    private void OnDisable()
    {
        // Unregister SceneView callback
        SceneView.duringSceneGui -= OnSceneGUI;
    }
    
    private void OnDestroy()
    {
        // Ensure callback is unregistered
        SceneView.duringSceneGui -= OnSceneGUI;
    }
    
    private void OnSceneGUI(SceneView sceneView)
    {
        if (!showVisualization || controller == null) return;
        
        serializedController.Update();
        
        // Get transform
        Transform transform = controller.transform;
        if (cachedTransformField != null)
        {
            Transform cachedTransform = cachedTransformField.GetValue(controller) as Transform;
            if (cachedTransform != null)
            {
                transform = cachedTransform;
            }
        }
        
        Vector3 rayStart = transform.position;
        float rideHeight = rideHeightProp.floatValue;
        float rayToGroundLength = rayToGroundLengthProp.floatValue;
        float groundSnapDistance = groundSnapDistanceProp.floatValue;
        
        // Get runtime data via reflection
        Vector3 groundNormal = Vector3.up;
        float groundAngle = 0f;
        bool isGrounded = controller.IsGrounded;
        
        if (groundNormalField != null)
        {
            object groundNormalObj = groundNormalField.GetValue(controller);
            if (groundNormalObj != null)
            {
                groundNormal = (Vector3)groundNormalObj;
            }
        }
        
        if (groundAngleField != null)
        {
            object groundAngleObj = groundAngleField.GetValue(controller);
            if (groundAngleObj != null)
            {
                groundAngle = (float)groundAngleObj;
            }
        }
        
        // Draw raycast line
        Handles.color = Color.yellow;
        Vector3 rayEnd = rayStart + (Vector3.down * rayToGroundLength);
        Handles.DrawLine(rayStart, rayEnd);
        
        if (showBasicInfo)
        {
            Handles.Label(rayEnd + Vector3.right * 0.2f, $"Ray Length: {rayToGroundLength:F2}");
        }
        
        // Draw ride height sphere
        Handles.color = Color.green;
        Vector3 rideHeightPos = rayStart + (Vector3.down * rideHeight);
        Handles.DrawWireDisc(rideHeightPos, Vector3.up, 0.1f);
        Handles.DrawWireDisc(rideHeightPos, Vector3.forward, 0.1f);
        Handles.DrawWireDisc(rideHeightPos, Vector3.right, 0.1f);
        
        if (showBasicInfo)
        {
            Handles.Label(rideHeightPos + Vector3.right * 0.2f, $"Ride Height: {rideHeight:F2}");
        }
        
        // Draw grounded tolerance sphere
        Handles.color = Color.cyan;
        Vector3 tolerancePos = rayStart + (Vector3.down * (rideHeight * GROUNDED_DISTANCE_TOLERANCE));
        Handles.DrawWireDisc(tolerancePos, Vector3.up, 0.1f);
        Handles.DrawWireDisc(tolerancePos, Vector3.forward, 0.1f);
        Handles.DrawWireDisc(tolerancePos, Vector3.right, 0.1f);
        
        if (showBasicInfo)
        {
            Handles.Label(tolerancePos + Vector3.right * 0.2f, $"Grounded Tolerance: {rideHeight * GROUNDED_DISTANCE_TOLERANCE:F2}");
        }
        
        // Draw ground snap distance if enabled
        if (groundSnapDistance > EPSILON_DISTANCE)
        {
            Handles.color = Color.magenta;
            Vector3 snapPos = rayStart + (Vector3.down * (rideHeight + groundSnapDistance));
            Handles.DrawWireDisc(snapPos, Vector3.up, 0.08f);
            Handles.DrawWireDisc(snapPos, Vector3.forward, 0.08f);
            Handles.DrawWireDisc(snapPos, Vector3.right, 0.08f);
            
            if (showBasicInfo)
            {
                Handles.Label(snapPos + Vector3.right * 0.2f, $"Ground Snap: {groundSnapDistance:F2}");
            }
        }
        
        // Draw ground normal if grounded
        if (isGrounded && groundAngle > EPSILON_DISTANCE)
        {
            Handles.color = Color.red;
            Vector3 normalEnd = rayStart + (groundNormal * 0.5f);
            Handles.DrawLine(rayStart, normalEnd);
            
            // Draw arrow head manually for better compatibility
            Vector3 arrowDir = groundNormal.normalized;
            Vector3 right = Vector3.Cross(arrowDir, Vector3.up);
            if (right.sqrMagnitude < 0.01f)
            {
                right = Vector3.Cross(arrowDir, Vector3.forward);
            }
            right.Normalize();
            Vector3 up = Vector3.Cross(right, arrowDir);
            float arrowSize = 0.1f;
            Vector3 arrowTip = normalEnd;
            Vector3 arrowBase = arrowTip - arrowDir * arrowSize;
            Handles.DrawLine(arrowTip, arrowBase + right * arrowSize * 0.5f);
            Handles.DrawLine(arrowTip, arrowBase - right * arrowSize * 0.5f);
            Handles.DrawLine(arrowTip, arrowBase + up * arrowSize * 0.5f);
            Handles.DrawLine(arrowTip, arrowBase - up * arrowSize * 0.5f);
            
            if (showBasicInfo)
            {
                Handles.Label(normalEnd + Vector3.right * 0.1f, $"Ground Normal\nAngle: {groundAngle:F1}°");
            }
        }
        
        // Draw current position
        Handles.color = Color.white;
        Handles.DrawWireDisc(rayStart, Vector3.up, 0.05f);
        Handles.DrawWireDisc(rayStart, Vector3.forward, 0.05f);
        Handles.DrawWireDisc(rayStart, Vector3.right, 0.05f);
        
        // Draw text information
        if (showBasicInfo || showAdvancedInfo)
        {
            DrawTextInformation(controller, rayStart, rideHeight, isGrounded, groundAngle);
        }
        
        serializedController.ApplyModifiedProperties();
    }
    
    private void DrawTextInformation(PhysicsBasedCharacterController controller, Vector3 position, float rideHeight, bool isGrounded, float groundAngle)
    {
        Vector3 labelOffset = Vector3.up * 0.5f;
        int lineIndex = 0;
        float lineHeight = 0.15f;
        
        if (showBasicInfo)
        {
            // Basic information
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"=== Basic Info ===");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Is Grounded: {(isGrounded ? "True" : "False")}");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Ride Height: {rideHeight:F2}");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Ray Length: {rayToGroundLengthProp.floatValue:F2}");
            
            if (groundAngle > EPSILON_DISTANCE)
            {
                Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                    $"Ground Angle: {groundAngle:F1}°");
            }
            
            Vector3 velocity = controller.Velocity;
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Velocity: ({velocity.x:F2}, {velocity.y:F2}, {velocity.z:F2})");
            
            Vector3 goalVelocity = controller.GoalVelocity;
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Goal Velocity: ({goalVelocity.x:F2}, {goalVelocity.y:F2}, {goalVelocity.z:F2})");
        }
        
        if (showAdvancedInfo)
        {
            lineIndex++;
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"=== Advanced Info ===");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Ride Spring Strength: {rideSpringStrengthProp.floatValue:F1}");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Ride Spring Damper: {rideSpringDamperProp.floatValue:F1}");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Upright Spring Strength: {uprightSpringStrengthProp.floatValue:F1}");
            
            Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                $"Upright Spring Damper: {uprightSpringDamperProp.floatValue:F1}");
            
            if (groundSnapDistanceProp.floatValue > EPSILON_DISTANCE)
            {
                Handles.Label(position + labelOffset + Vector3.up * (lineIndex++ * lineHeight), 
                    $"Ground Snap Distance: {groundSnapDistanceProp.floatValue:F2}");
            }
        }
    }
}
#endif
