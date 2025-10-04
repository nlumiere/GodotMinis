using System.Runtime.CompilerServices;
using Godot;

public partial class Bird : CharacterBody3D
{
	private readonly float DEADZONE = 0.2f;
	private const float ForwardThrust = 1000.0f;
	private const float Gravity = 60.0f;
	private const float DragCoefficient = 0.02f;
	private const float FlapDuration = 0.3f;
	private float _flapLockout = 0.0f;
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
	private const float AngularDamping = 0.5f;
	private const float MaxBankAngle = 45.0f;
	private float _currentBankAngle = 0.0f;
	private const float BankingSpeed = 90.0f;
	private const float ControlTorque = 15.0f;
	private const float AngularMass = 1.5f;

	private bool _wingsTucked = false;
	private const float UprightSpeed = 3.0f;
	private float _cameraPanAngle = 0.0f;
	private const float CameraPanSpeed = 90.0f;
	private const float CameraDistance = 10.0f;
	private const float CameraHeight = 4.0f;
	private float _previousCameraPanAngle = 0.0f;
	private float _baseUnlockedAngle = 180.0f;
	private bool _isUsingGamepad = false;

	private RichTextLabel _heightLabel;
	private RichTextLabel _speedLabel;
	private bool _knownHeight = false;
	private double _lastComputedLabelsTime = 0.0f;

	public override void _Ready()
	{
		_isUsingGamepad = Input.GetConnectedJoypads().Count > 0;
		Input.JoyConnectionChanged += UpdateOnControllerChanged;
		UpdateControllerInfo();

		FloorMaxAngle = Mathf.DegToRad(90.0f);
		FloorSnapLength = 0.1f;

		_camera = GetNode<Camera3D>("Camera3D");

		_heightLabel = GetNode<RichTextLabel>("RichTextLabel2");
		_speedLabel = GetNode<RichTextLabel>("RichTextLabel3");

		if (_camera != null)
		{
			Vector3 birdPosition = GlobalPosition;
			float defaultAngle = 180.0f;
			Vector3 offsetDirection = new Vector3(
				Mathf.Sin(Mathf.DegToRad(defaultAngle)),
				0,
				Mathf.Cos(Mathf.DegToRad(defaultAngle))
			);
			Vector3 targetCameraPos = birdPosition + offsetDirection * CameraDistance;
			targetCameraPos.Y = birdPosition.Y + CameraHeight;
			_camera.GlobalPosition = targetCameraPos;
			_camera.LookAt(birdPosition, Vector3.Up);
		}
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
		UpdateWingVisibility();
		MoveAndSlide();
		UpdateCamera();
		UpdateHUD(delta);
	}

	private void HandleInput(double delta)
	{
		_flapLockout -= (float)delta;
		Vector3 velocity = Velocity;

		float turnInput = _isUsingGamepad ? -Input.GetJoyAxis(0, JoyAxis.LeftX) : Input.GetAxis("ui_left", "ui_right");
		float pitchInput = _isUsingGamepad ? -Input.GetJoyAxis(0, JoyAxis.LeftY) : Input.GetAxis("ui_up", "ui_down");
		if (Mathf.Abs(turnInput) < DEADZONE) turnInput = 0;
		if (Mathf.Abs(pitchInput) < DEADZONE) pitchInput = 0;

		if (_tail != null)
		{
			float maxTailDeflection = 30.0f;
			float targetTailYaw = turnInput * maxTailDeflection;
			float targetTailPitch = pitchInput * maxTailDeflection;
			_tail.Rotation = new Vector3(Mathf.DegToRad(targetTailPitch), 0, Mathf.DegToRad(targetTailYaw));
		}

		ApplyVisualBanking(turnInput, (float)delta);

		float cameraPanInput = _isUsingGamepad ? Input.GetJoyAxis(0, JoyAxis.RightX) : Input.GetAxis("ui_text_caret_left", "ui_text_caret_right"); // If using a gamepad, use axis, otherwise left and right arrows
		if (Mathf.Abs(cameraPanInput) < DEADZONE) cameraPanInput = 0;
		_cameraPanAngle += cameraPanInput * CameraPanSpeed * (float)delta;

		if (Input.IsActionJustPressed("right_stick_pressed"))
		{
			_cameraPanAngle = 0.0f;

			if (_camera != null)
			{
				Vector3 birdPosition = GlobalPosition;
				Vector3 offsetDirection;

				if (IsOnFloor())
				{
					// When on ground, use a reliable default direction to avoid rotation issues
					offsetDirection = new Vector3(0, 0, 1); // Default behind bird in world space
				}
				else
				{
					// When flying, use bird's orientation
					Transform3D pitchedTransform = Transform;
					pitchedTransform = pitchedTransform.Rotated(Transform.Basis.X, Mathf.DegToRad(90.0f));
					Vector3 birdForward = -pitchedTransform.Basis.Z;
					Vector3 horizontalForward = new Vector3(birdForward.X, 0, birdForward.Z);

					if (horizontalForward.LengthSquared() < 0.01f)
					{
						offsetDirection = new Vector3(0, 0, 1);
					}
					else
					{
						offsetDirection = -horizontalForward.Normalized();
					}
				}

				Vector3 targetCameraPos = birdPosition + offsetDirection * CameraDistance;
				targetCameraPos.Y = birdPosition.Y + CameraHeight;
				_camera.GlobalPosition = targetCameraPos;
				_camera.LookAt(birdPosition, Vector3.Up);
			}
		}

		_wingsTucked = Input.IsActionPressed("ui_select");

		if (Input.IsActionPressed("ui_accept") && _flapLockout <= 0)
		{
			_wingsTucked = false;

			Transform3D pitchedTransform = Transform;
			pitchedTransform = pitchedTransform.Rotated(Transform.Basis.X, Mathf.DegToRad(90.0f));

			Vector3 pitchedForward = -pitchedTransform.Basis.Z;
			pitchedForward = pitchedForward.Normalized();
			Vector3 forwardForce = pitchedForward * ForwardThrust * (float)delta;
			velocity += forwardForce;

			Vector3 pitchedUp = -pitchedTransform.Basis.Z;
			const float FlapLift = 400.0f;
			Vector3 liftForce = pitchedUp * FlapLift * (float)delta;
			velocity += liftForce;

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
			Vector3 birdForward = -Transform.Basis.Z;
			Vector3 birdUp = Transform.Basis.Y;
			float airspeed = velocity.Length();
			const float MinLiftSpeed = 20.0f;

			if (airspeed > MinLiftSpeed && !_wingsTucked)
			{
				Vector3 velocityDirection = velocity.Normalized();
				float angleOfAttack = Mathf.Asin(velocityDirection.Dot(birdUp));
				float speedFactor = (airspeed - MinLiftSpeed) / 12.0f;
				float lowSpeedFalloff = Mathf.SmoothStep(0.0f, MinLiftSpeed * 2, (airspeed - MinLiftSpeed) / 5.0f);
				speedFactor *= lowSpeedFalloff;
				// .8 * <under 100> + .9 * <100-200> + 1.0 * <200+>
				float liftCoefficient = 0.8f * Mathf.Clamp(airspeed / 100.0f, 0.0f, 1.0f) +
									 0.1f * Mathf.Clamp((airspeed - 100.0f) / 100.0f, 0.0f, 1.0f) +
									 0.1f * Mathf.Clamp((airspeed - 200.0f) / 100.0f, 0.0f, 1.0f);
				float liftMagnitude = liftCoefficient * speedFactor * airspeed * Mathf.Sin(angleOfAttack);
				liftMagnitude = Mathf.Clamp(liftMagnitude, -Gravity * 0.7f, Gravity * 1.0f);

				Vector3 liftDirection = birdUp - velocityDirection * velocityDirection.Dot(birdUp);
				if (liftDirection.Length() > 0.01f)
				{
					liftDirection = liftDirection.Normalized();
					Vector3 liftForce = liftDirection * liftMagnitude * (float)delta;
					velocity += liftForce;
				}
			}

			float gravityMultiplier = _wingsTucked ? 1.3f : 1.0f;
			velocity.Y -= Gravity * gravityMultiplier * (float)delta;
		}
		else
		{
			if (velocity.Y < 0)
			{
				velocity.Y = 0;
			}

			Vector3 horizontalVelocity = new Vector3(velocity.X, 0, velocity.Z);
			float horizontalSpeed = horizontalVelocity.Length();

			if (horizontalSpeed > 0.1f)
			{
				const float ImpactFriction = 35.0f;
				float frictionReduction = ImpactFriction * (float)delta;
				float newHorizontalSpeed = Mathf.Max(0.0f, horizontalSpeed - frictionReduction);
				float speedRatio = newHorizontalSpeed / horizontalSpeed;
				velocity.X *= speedRatio;
				velocity.Z *= speedRatio;
			}
			else
			{
				velocity.X = 0;
				velocity.Z = 0;
				_angularVelocity.Y = 0;
			}

			_angularVelocity.Y *= (1.0f - 12.0f * (float)delta);
		}

		ApplyRudderPhysics((float)delta);
		ApplyGroundStabilization((float)delta);
		ApplyQuadraticDrag(ref velocity, (float)delta);

		Velocity = velocity;
	}

	private void StartWingFlap()
	{
		_isFlapping = true;
		_flapAnimationTime = 0.0f;
	}

	private void UpdateWingVisibility()
	{
		if (_wingL != null)
		{
			_wingL.Visible = !_wingsTucked;
		}

		if (_wingR != null)
		{
			_wingR.Visible = !_wingsTucked;
		}
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

	private void ApplyGroundStabilization(float delta)
	{
		if (!IsOnFloor()) return;

		Vector3 currentEuler = Transform.Basis.GetEuler();
		float currentRoll = currentEuler.Z;
		float currentPitch = currentEuler.X;

		bool isActiveTurning = false;
		if (_tail != null)
		{
			float tailYaw = _tail.Rotation.Z;
			float tailPitch = _tail.Rotation.X;
			isActiveTurning = Mathf.Abs(tailYaw) > 0.05f || Mathf.Abs(tailPitch) > 0.05f;
		}

		if (!isActiveTurning)
		{
			float targetRoll = 0.0f;
			float targetPitch = 0.0f;
			float newRoll = Mathf.LerpAngle(currentRoll, targetRoll, UprightSpeed * delta);
			float newPitch = Mathf.LerpAngle(currentPitch, targetPitch, UprightSpeed * delta);
			Vector3 newEuler = new Vector3(newPitch, currentEuler.Y, newRoll);
			Transform = new Transform3D(Basis.FromEuler(newEuler), GlobalPosition);
		}
	}

	private void ApplyRudderPhysics(float delta)
	{
		if (_tail == null) return;

		float tailYaw = _tail.Rotation.Z;
		float tailPitch = _tail.Rotation.X;
		float airspeed = Velocity.Length();

		float speedEffectiveness = Mathf.Min(1.0f, airspeed / 8.0f);
		speedEffectiveness = Mathf.Max(0.1f, speedEffectiveness);

		Vector3 currentPosition = GlobalPosition;
		Basis currentBasis = Transform.Basis;

		// PITCH: Direct control with non-linear response curve
		if (Mathf.Abs(tailPitch) > 0.001f)
		{
			float normalizedPitch = tailPitch / Mathf.DegToRad(30.0f);
			float pitchCurve = normalizedPitch * normalizedPitch * Mathf.Sign(normalizedPitch);
			float enhancedPitch = pitchCurve * Mathf.DegToRad(30.0f);

			float pitchRate = enhancedPitch * 8.0f * speedEffectiveness * delta;
			Vector3 pitchAxis = Transform.Basis.X;
			currentBasis = currentBasis.Rotated(pitchAxis, pitchRate);
		}

		Vector3 yawTorque = Vector3.Zero;
		yawTorque.Y = tailYaw * (ControlTorque * 0.4f) * speedEffectiveness;

		Vector3 yawAngularAcceleration = yawTorque / AngularMass;
		_angularVelocity.Y += yawAngularAcceleration.Y * delta;
		_angularVelocity.Y *= (1.0f - AngularDamping * delta);

		if (Mathf.Abs(tailYaw) < 0.05f)
		{
			float stabilizingForce = 3.0f;
			_angularVelocity.Y *= (1.0f - stabilizingForce * delta);
		}

		float maxYawRate = Mathf.Lerp(0.9f, 1.2f, Mathf.Min(1.0f, airspeed / 20.0f));
		_angularVelocity.Y = Mathf.Clamp(_angularVelocity.Y, -maxYawRate, maxYawRate);

		if (Mathf.Abs(_angularVelocity.Y) > 0.001f)
		{
			currentBasis = currentBasis.Rotated(Vector3.Up, _angularVelocity.Y * delta);
		}

		Transform = new Transform3D(currentBasis, currentPosition);
	}

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

	private void ApplyQuadraticDrag(ref Vector3 velocity, float delta)
	{
		float speed = velocity.Length();
		if (speed > 0.01f)
		{
			Transform3D pitchedTransform = Transform;
			pitchedTransform = pitchedTransform.Rotated(Transform.Basis.X, Mathf.DegToRad(90.0f));
			Vector3 birdForward = -pitchedTransform.Basis.Z;
			Vector3 birdRight = pitchedTransform.Basis.X;
			Vector3 birdUp = pitchedTransform.Basis.Y;

			float forwardVel = velocity.Dot(birdForward);
			float rightVel = velocity.Dot(birdRight);
			float upVel = velocity.Dot(birdUp);

			float dragMultiplier = _wingsTucked ? 0.1f : 1.0f;
			float forwardDragRate = DragCoefficient * 0.15f * dragMultiplier;
			float sideDragRate = DragCoefficient * 2.0f * dragMultiplier;
			float verticalDragRate = DragCoefficient * 1f * dragMultiplier;

			float forwardDragAccel = -forwardDragRate * forwardVel * Mathf.Abs(forwardVel);
			float rightDragAccel = -sideDragRate * rightVel * Mathf.Abs(rightVel);
			float upDragAccel = -verticalDragRate * upVel * Mathf.Abs(upVel);
			Vector3 dragAcceleration = (birdForward * forwardDragAccel +
										birdRight * rightDragAccel +
										birdUp * upDragAccel) * delta;

			velocity += dragAcceleration;
		}
	}

	/// <summary>
	/// Maintains camera position at fixed distance from bird at set height
	/// </summary>
	private void UpdateCamera()
	{
		if (_camera != null)
		{
			Vector3 birdPosition = GlobalPosition;

			if (_cameraPanAngle == 0.0f)
			{
				// Locked camera: Follow behind bird smoothly
				Vector3 currentCameraOffset = _camera.GlobalPosition - birdPosition;
				currentCameraOffset.Y = 0; // Keep only horizontal component for following

				// Use the same forward direction as the physics system for consistency
				Transform3D pitchedTransform = Transform;
				pitchedTransform = pitchedTransform.Rotated(Transform.Basis.X, Mathf.DegToRad(90.0f));
				Vector3 birdForward = -pitchedTransform.Basis.Z;
				Vector3 horizontalForward = new Vector3(birdForward.X, 0, birdForward.Z);

				Vector3 idealOffset;
				if (horizontalForward.LengthSquared() < 0.01f)
				{
					// Bird is vertical - maintain current horizontal position
					idealOffset = currentCameraOffset.Normalized() * CameraDistance;
				}
				else
				{
					// Bird has horizontal movement - follow behind
					idealOffset = -horizontalForward.Normalized() * CameraDistance;
				}

				// Smooth interpolation towards ideal position (horizontal only)
				float lerpSpeed = 1.5f;
				Vector3 targetOffset = currentCameraOffset.Lerp(idealOffset, lerpSpeed * (float)GetPhysicsProcessDeltaTime());
				targetOffset = targetOffset.Normalized() * CameraDistance;

				Vector3 targetCameraPos = birdPosition + targetOffset;
				targetCameraPos.Y = birdPosition.Y + CameraHeight;
				_camera.GlobalPosition = targetCameraPos;
			}
			else
			{
				if (_previousCameraPanAngle == 0.0f && _cameraPanAngle != 0.0f)
				{
					Vector3 currentOffset = _camera.GlobalPosition - birdPosition;
					currentOffset.Y = 0;
					if (currentOffset.LengthSquared() > 0.01f)
					{
						float currentAngle = Mathf.Atan2(currentOffset.X, currentOffset.Z) * 180.0f / Mathf.Pi;
						_baseUnlockedAngle = currentAngle;
					}
				}

				float totalAngle = _cameraPanAngle + _baseUnlockedAngle;
				Vector3 offsetDirection = new Vector3(
					Mathf.Sin(Mathf.DegToRad(totalAngle)),
					0,
					Mathf.Cos(Mathf.DegToRad(totalAngle))
				);
				Vector3 targetCameraPos = birdPosition + offsetDirection * CameraDistance;
				targetCameraPos.Y = birdPosition.Y + CameraHeight;
				_camera.GlobalPosition = targetCameraPos;
			}

			_previousCameraPanAngle = _cameraPanAngle;

			Vector3 lookDirection = (birdPosition - _camera.GlobalPosition).Normalized();
			Vector3 upVector = Vector3.Up;

			if (Mathf.Abs(lookDirection.Dot(upVector)) > 0.99f)
			{
				upVector = Vector3.Forward;
			}

			_camera.LookAt(birdPosition, upVector);
		}
	}

	private void UpdateHUD(double delta)
	{
		_lastComputedLabelsTime += delta;
		if (_lastComputedLabelsTime < 0.2f) return;
		_lastComputedLabelsTime = 0.0f;

		if (_heightLabel != null)
		{
			float height = GetHeightAboveGround();
			_heightLabel.Text = $"Height: {height:F1}" + (_knownHeight ? "" : "?" + " units");
		}

		if (_speedLabel != null)
		{
			_speedLabel.Text = $"Speed: {Velocity.Length():F1} units/s";
		}
	}

	private float GetHeightAboveGround()
	{
		PhysicsDirectSpaceState3D spaceState = GetWorld3D().DirectSpaceState;
		Vector3 from = GlobalPosition;
		Vector3 to = from + Vector3.Down * 1000.0f; // Cast ray 1000 units down

		PhysicsRayQueryParameters3D query = PhysicsRayQueryParameters3D.Create(from, to);
		query.CollideWithAreas = false;
		query.CollideWithBodies = true;
		query.CollisionMask = 0xFFFFFFFF; // Check all collision layers

		Godot.Collections.Dictionary result = spaceState.IntersectRay(query);

		if (result.Count > 0)
		{
			Vector3 hitPosition = (Vector3)result["position"];
			_knownHeight = true;
			return GlobalPosition.Y - hitPosition.Y;
		}

		_knownHeight = false;
		return GlobalPosition.Y;
	}

	private void UpdateOnControllerChanged(long device, bool connected)
	{
		UpdateControllerInfo();
	}

	private void UpdateControllerInfo()
	{
		// Update the gamepad status based on current connected joypads
		_isUsingGamepad = Input.GetConnectedJoypads().Count > 0;
		
		RichTextLabel gamepadConnected = GetNode<RichTextLabel>("RichTextLabel");
		gamepadConnected.Text = _isUsingGamepad ? "Gamepad connected" : "No gamepad detected";
		gamepadConnected.Modulate = _isUsingGamepad ? Colors.Green : Colors.Red;
		
		RichTextLabel controlsInfo = GetNode<RichTextLabel>("RichTextLabel4");
		controlsInfo.Text = _isUsingGamepad ?
			"Left Stick: Pitch/Roll\nRight Stick: Pan Camera\nA: Flap\nX: Tuck Wings\nRight Stick Click: Reset Camera" :
			"WASD: Pitch/Roll\nLeft/Right: Pan Camera\nSpace: Flap\nEnter: Tuck Wings\nUp: Reset Camera";
	}
}
