using UnityEngine;

public class CharacterControllerAudioVisuals : MonoBehaviour
{
    [SerializeField] private PhysicsBasedCharacterController _controller;
    [SerializeField] private ParticleSystem _dustParticleSystem;
    
    private ParticleSystem.EmissionModule _emission;
    private AudioManager _audioManager;

    private void Awake()
    {
        if (_controller == null)
        {
            _controller = GetComponent<PhysicsBasedCharacterController>();
        }

        _audioManager = FindObjectOfType<AudioManager>();

        if (_dustParticleSystem != null)
        {
            _emission = _dustParticleSystem.emission;
            _emission.enabled = false;
        }
    }

    private void OnEnable()
    {
        _controller.OnJump.AddListener(PlayJumpSound);
        _controller.OnLand.AddListener(PlayLandSound);
        _controller.OnMoveStarted.AddListener(StartWalkingSoundAndParticles);
        _controller.OnMoveStopped.AddListener(StopWalkingSoundAndParticles);
        _controller.OnUngrounded.AddListener(StopWalkingSoundAndParticles);
    }

    private void OnDisable()
    {
        _controller.OnJump.RemoveListener(PlayJumpSound);
        _controller.OnLand.RemoveListener(PlayLandSound);
        _controller.OnMoveStarted.RemoveListener(StartWalkingSoundAndParticles);
        _controller.OnMoveStopped.RemoveListener(StopWalkingSoundAndParticles);
        _controller.OnUngrounded.RemoveListener(StopWalkingSoundAndParticles);
    }

    private void PlayJumpSound()
    {
        if (_audioManager != null)
        {
            _audioManager.Play("Jump");
        }
    }

    private void PlayLandSound()
    {
        if (_audioManager != null)
        {
            if (!_audioManager.IsPlaying("Land"))
            {
                _audioManager.Play("Land");
            }
        }
    }

    private void StartWalkingSoundAndParticles()
    {
        if (_audioManager != null)
        {
            if (!_audioManager.IsPlaying("Walking"))
            {
                _audioManager.Play("Walking");
            }
        }

        if (_dustParticleSystem != null)
        {
            _emission.enabled = true;
        }
    }

    private void StopWalkingSoundAndParticles()
    {
        if (_audioManager != null)
        {
            _audioManager.Stop("Walking");
        }

        if (_dustParticleSystem != null)
        {
            _emission.enabled = false;
        }
    }
}
