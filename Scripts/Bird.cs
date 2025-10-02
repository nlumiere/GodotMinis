using Godot;

public partial class Bird : CharacterBody3D
{
	private readonly float DEADZONE = 0.2f;
	private const float FlapForce = 600.0f;
	private const float ForwardThrust = 1200.0f;
	private const float TurnSpeed = 120.0f;
	private const float Gravity = 15.0f;
	private const float DragCoefficient = 0.02f;
	private const float FlapDuration = 0.3f;
	private float _flapLockout = 0.0f;
	private float _previousYRotation = 0.0f;
	private Camera3D _camera;
	private Node3D _body;
	private Node3D _tail;
	private Node3D _wingL;
	private Node3D _wingR;
	
	private float _flapAnimationTime = 0.0f;
	private bool _isFlapping = false;
	private const float FlapAngle = 60.0f;
	private Vector3 _wingLBaseRotation;
	private Vector3 _wingRBaseRotation;
	private Vector3 _wingLBasePosition;
	private Vector3 _wingRBasePosition;
	
	private Vector3 _angularVelocity = Vector3.Zero;
	private const float AngularDamping = 3.0f;
	private const float MaxBankAngle = 45.0f;
	private float _currentBankAngle = 0.0f;
	private const float BankingSpeed = 90.0f;

	public override void _Ready()
	{
		FloorMaxAngle = Mathf.DegToRad(90.0f);
		FloorSnapLength = 0.1f;

		_camera = GetNode<Camera3D>("Camera3D");
		_body = GetNode<Node3D>("Body");
		_tail = _body.GetNode<Node3D>("TailPivot");
		_wingL = _body.GetNode<Node3D>("WingL");
		_wingR = _body.GetNode<Node3D>("WingR");
		
		if (_wingL != null) 
		{
			_wingLBaseRotation = _wingL.Rotation;
			_wingLBasePosition = _wingL.Position;
		}
		if (_wingR != null) 
		{
			_wingRBaseRotation = _wingR.Rotation;
			_wingRBasePosition = _wingR.Position;
		}
		
		if (_tail != null)
		{
			Vector3 originalPos = _tail.Position;
			_tail.Position = originalPos;
		}
	}

	public override void _PhysicsProcess(double delta)
	{
		HandleInput(delta);
		UpdateMovement(delta);
		UpdateWingAnimation((float)delta);
		MoveAndSlide();
		UpdateCamera();
	}

	private void HandleInput(double delta)
	{
		_flapLockout -= (float)delta;
		Vector3 velocity = Velocity;

		float turnInput = -Input.GetJoyAxis(0, JoyAxis.LeftX);
		float pitchInput = -Input.GetJoyAxis(0, JoyAxis.LeftY);
		if (Mathf.Abs(turnInput) < DEADZONE) turnInput = 0;
		if (Mathf.Abs(pitchInput) < DEADZONE) pitchInput = 0;

		if (_tail != null)
		{
			float maxTailDeflection = 30.0f;
			float targetTailYaw = turnInput * maxTailDeflection;
			float targetTailPitch = pitchInput * maxTailDeflection;
			
			// X rotation = pitch, Z rotation = yaw for capsule oriented along Z-axis
			_tail.Rotation = new Vector3(Mathf.DegToRad(targetTailPitch), 0, Mathf.DegToRad(targetTailYaw));
		}
		
		ApplyVisualBanking(turnInput, (float)delta);

		// Thrust applied 90 degrees upward from bird's current orientation
		if (Input.IsActionPressed("ui_accept") && _flapLockout <= 0)
		{
			GD.Print("Thrust!");
			Transform3D pitchedTransform = Transform;
			pitchedTransform = pitchedTransform.Rotated(Transform.Basis.X, Mathf.DegToRad(90.0f));
			Vector3 pitchedForward = -pitchedTransform.Basis.Z;
			pitchedForward = pitchedForward.Normalized();
			velocity += pitchedForward * ForwardThrust * (float)delta;
			_flapLockout = FlapDuration;
			StartWingFlap();
		}

		Velocity = velocity;
	}

	private void UpdateMovement(double delta)
	{
		Vector3 velocity = Velocity;

		if (!IsOnFloor())
		{
			float effectiveGravity = Gravity;
			
			Vector3 birdForward = -Transform.Basis.Z;
			float forwardSpeed = velocity.Dot(birdForward);
			
			// Simple lift model: reduce gravity based on forward speed
			if (forwardSpeed > 1.0f)
			{
				float liftFactor = Mathf.Min(0.95f, (forwardSpeed * forwardSpeed) * 0.1f);
				effectiveGravity = Gravity * (1.0f - liftFactor);
			}
			
			velocity.Y -= effectiveGravity * (float)delta;
		}
		else if (velocity.Y < 0)
		{
			velocity.Y = 0;
		}

		ApplyRudderPhysics((float)delta);
		ApplyQuadraticDrag(ref velocity, (float)delta);

		Velocity = velocity;
	}

	private void StartWingFlap()
	{
		_isFlapping = true;
		_flapAnimationTime = 0.0f;
	}

	private void UpdateWingAnimation(float delta)
	{
		if (!_isFlapping) return;
		
		_flapAnimationTime += delta;
		
		if (_flapAnimationTime >= FlapDuration)
		{
			_isFlapping = false;
			if (_wingL != null) 
			{
				_wingL.Rotation = _wingLBaseRotation;
				_wingL.Position = _wingLBasePosition;
			}
			if (_wingR != null) 
			{
				_wingR.Rotation = _wingRBaseRotation;
				_wingR.Position = _wingRBasePosition;
			}
			return;
		}
		
		// Three-phase realistic bird flap pattern
		float flapProgress = _flapAnimationTime / FlapDuration;
		float flapIntensity;
		
		if (flapProgress < 0.2f)
		{
			float windupProgress = flapProgress / 0.2f;
			flapIntensity = -0.3f * Mathf.Sin(windupProgress * Mathf.Pi * 0.5f);
		}
		else if (flapProgress < 0.6f)
		{
			float downstrokeProgress = (flapProgress - 0.2f) / 0.4f;
			flapIntensity = 1.0f * Mathf.Sin(downstrokeProgress * Mathf.Pi);
		}
		else
		{
			float recoveryProgress = (flapProgress - 0.6f) / 0.4f;
			flapIntensity = 1.0f * (1.0f - recoveryProgress);
		}
		
		float flapAngleL = -flapIntensity * Mathf.DegToRad(FlapAngle);
		float flapAngleR = flapIntensity * Mathf.DegToRad(FlapAngle);
		
		if (_wingL != null) 
		{
			_wingL.Rotation = _wingLBaseRotation + new Vector3(0, 0, flapAngleL);
			Vector3 offsetFromBody = _wingLBasePosition;
			Vector3 rotatedOffset = offsetFromBody.Rotated(Vector3.Forward, flapAngleL);
			_wingL.Position = rotatedOffset;
		}
		if (_wingR != null) 
		{
			_wingR.Rotation = _wingRBaseRotation + new Vector3(0, 0, flapAngleR);
			Vector3 offsetFromBody = _wingRBasePosition;
			Vector3 rotatedOffset = offsetFromBody.Rotated(Vector3.Forward, flapAngleR);
			_wingR.Position = rotatedOffset;
		}
	}

	/// <summary>
	/// Applies rotation based on tail deflection, using speed-dependent control authority
	/// to avoid gimbal lock issues while maintaining realistic flight physics
	/// </summary>
	private void ApplyRudderPhysics(float delta)
	{
		if (_tail == null) return;
		
		float tailYaw = Mathf.RadToDeg(_tail.Rotation.Z);
		float tailPitch = Mathf.RadToDeg(_tail.Rotation.X);
		float currentSpeed = Velocity.Length();
		
		// Different minimum control authority for yaw vs pitch when stationary
		float yawSpeedFactor, pitchSpeedFactor;
		if (currentSpeed < 1.0f)
		{
			yawSpeedFactor = 0.1f;
			pitchSpeedFactor = 0.6f;
		}
		else
		{
			float baseFactor = (currentSpeed * currentSpeed * currentSpeed) / (20.0f * 20.0f * 20.0f);
			yawSpeedFactor = Mathf.Max(0.2f, Mathf.Min(1.0f, baseFactor));
			pitchSpeedFactor = Mathf.Max(0.6f, Mathf.Min(1.0f, baseFactor));
		}
		
		float rudderAuthority = 2.0f;
		float yawRate = tailYaw * rudderAuthority * yawSpeedFactor * delta;
		float pitchRate = tailPitch * rudderAuthority * pitchSpeedFactor * delta * 4;
		
		// Use basis rotation to avoid gimbal lock, keeping bird's position fixed
		Vector3 currentPosition = GlobalPosition;
		Basis currentBasis = Transform.Basis;
		
		if (Mathf.Abs(yawRate) > 0.001f)
		{
			currentBasis = currentBasis.Rotated(Vector3.Up, Mathf.DegToRad(yawRate));
		}
		
		if (Mathf.Abs(pitchRate) > 0.001f)
		{
			Vector3 birdRightAxis = Transform.Basis.X;
			currentBasis = currentBasis.Rotated(birdRightAxis, Mathf.DegToRad(pitchRate));
		}
		
		Transform = new Transform3D(currentBasis, currentPosition);
	}

	/// <summary>
	/// Applies visual banking to the bird's body during turns without affecting physics
	/// </summary>
	private void ApplyVisualBanking(float turnInput, float delta)
	{
		float targetBankAngle = turnInput * MaxBankAngle;
		float bankingRate = BankingSpeed * delta;
		_currentBankAngle = Mathf.MoveToward(_currentBankAngle, targetBankAngle, bankingRate);
		
		if (_body != null)
		{
			Vector3 bodyRotation = _body.Rotation;
			bodyRotation.Y = -Mathf.DegToRad(_currentBankAngle);
			_body.Rotation = bodyRotation;
		}
	}

	/// <summary>
	/// Applies quadratic air drag that increases with velocity squared
	/// </summary>
	private void ApplyQuadraticDrag(ref Vector3 velocity, float delta)
	{
		float speed = velocity.Length();
		if (speed > 0.01f)
		{
			float dragMagnitude = DragCoefficient * speed * speed;
			Vector3 dragDirection = -velocity.Normalized();
			Vector3 dragAcceleration = dragDirection * dragMagnitude;
			Vector3 newVelocity = velocity + dragAcceleration * delta;
			
			// Ensure drag doesn't reverse direction
			if (newVelocity.Dot(velocity) < 0)
			{
				velocity = Vector3.Zero;
			}
			else
			{
				velocity = newVelocity;
			}
		}
	}

	/// <summary>
	/// Maintains camera position at fixed distance from bird with safe LookAt to avoid gimbal lock
	/// </summary>
	private void UpdateCamera()
	{
		if (_camera != null)
		{
			Vector3 birdPosition = GlobalPosition;
			Vector3 cameraToBird = birdPosition - _camera.GlobalPosition;
			cameraToBird.Y = 0;
			
			if (cameraToBird.LengthSquared() < 0.01f)
			{
				cameraToBird = new Vector3(0, 0, 10);
			}
			
			Vector3 normalizedOffset = cameraToBird.Normalized() * 10.0f;
			Vector3 targetCameraPos = birdPosition - normalizedOffset;
			targetCameraPos.Y = birdPosition.Y + 4.0f;
			
			_camera.GlobalPosition = targetCameraPos;
			
			// Safe LookAt to avoid gimbal lock when looking straight up/down
			Vector3 lookDirection = (birdPosition - _camera.GlobalPosition).Normalized();
			Vector3 upVector = Vector3.Up;
			
			if (Mathf.Abs(lookDirection.Dot(upVector)) > 0.99f)
			{
				upVector = Vector3.Forward;
			}
			
			_camera.LookAt(birdPosition, upVector);
		}
	}

	private void ApplyBankedTurnPhysics(ref Vector3 velocity, float delta)
	{
		if (_tail == null) return;
		
		float tailYawDeflection = Mathf.RadToDeg(_tail.Rotation.Z);
		float bankingStrength = tailYawDeflection / 45.0f;

		Vector3 birdForward = -Transform.Basis.Z;
		Vector3 birdRight = Transform.Basis.X;
		float totalSpeed = velocity.Length();

		float bankingForce = bankingStrength * totalSpeed * 0.5f;
		Vector3 bankingVelocity = birdRight * bankingForce * delta;
		
		velocity += bankingVelocity;
	}
}
