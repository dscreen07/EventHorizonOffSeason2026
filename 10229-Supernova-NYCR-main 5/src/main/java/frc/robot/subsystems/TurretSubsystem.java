package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    // Hardware
    private final TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_ID, "Colby"); // Your CAN ID
    
    // The Phoenix 6 Control Request
    private final MotionMagicVoltage motionMagicReq = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);
    
    private double targetAngleDegrees = 0.0;

    public TurretSubsystem(CommandSwerveDrivetrain drivetrain) {
      this.drivetrain = drivetrain;
      configureHardware();
    }

    public void configureHardware() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      // 1. Current Limits - Critical for Turrets
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 30.0; //Amps

      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 15.0; // Turrets don't need much battery power 20 before

      // 2. Soft Limits
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kForwardLimit;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.kReverseLimit;

      // 3. Tell motor to use internal sensor
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      // Rations (Required for Fused CAN)
      config.Feedback.SensorToMechanismRatio = TurretConstants.kTurretGearRatio; // Motor to CANcoder ratio

      // 4. Brake Mode
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      config.Slot0.kP = TurretConstants.kTurretkP;
      config.Slot0.kI = TurretConstants.kTurretkI;
      config.Slot0.kD = TurretConstants.kTurretkD;
      config.Slot0.kV = TurretConstants.kTurretkV;
      config.Slot0.kS = TurretConstants.kTurretkS;

      config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretCruiseVelocity;
      config.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretAcceleration;
      config.MotionMagic.MotionMagicJerk = TurretConstants.kTurretJerk;


      // Optimize CAN Bus
      turretMotor.getPosition().setUpdateFrequency(100);
      turretMotor.getVelocity().setUpdateFrequency(100);

      turretMotor.getConfigurator().apply(config);
      turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
      // Limelight is statically mounted to chassis. 
      // MegaTag2 handles physical offset natively via web interface.
    }

    public void trackLocationDynamic(Translation2d targetLocation, double timeOfFlight) {
      var state = drivetrain.getState();
      Pose2d robotPose = state.Pose;
      ChassisSpeeds robotVel = state.Speeds;

      // 1. Convert Robot-Relative Speeds to Field-Relative Speeds!
      Translation2d fieldVelocity = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond)
                                        .rotateBy(robotPose.getRotation());

      // 2. Compensate for drift using Field-Relative speeds
      double driftX = fieldVelocity.getX() * timeOfFlight;
      double driftY = fieldVelocity.getY() * timeOfFlight;

      double relativeX = targetLocation.getX() - robotPose.getX() - driftX;
      double relativeY = targetLocation.getY() - robotPose.getY() - driftY;

      // 3. Calculate local angle (Wrapped in negative sign to invert WPILib CCW+ for your CW+ motor)
      Rotation2d fieldRelativeTarget = new Rotation2d(relativeX, relativeY);
      targetAngleDegrees = -(fieldRelativeTarget.minus(robotPose.getRotation()).getDegrees());
      
      // 4. Send to Motor using Motion Magic
      double targetRotations = targetAngleDegrees / 360.0;
      turretMotor.setControl(motionMagicReq.withPosition(targetRotations));
    }

    public double getTurretAngleDegrees() {
      // Read the internal rotor position and convert back to degrees
      return turretMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean isAimed() {
      return Math.abs(getTurretAngleDegrees() - targetAngleDegrees) < TurretConstants.TURRET_TOLERANCE;
    }

    /**
     * Drives the turret manually using open-loop percent output.
     * @param speed from -1.0 to 1.0
     */
    public void rotateTurret(double speed) {
        turretMotor.setControl(manualRequest.withOutput(speed));
    }

    public void stop() {
      // Lock it at its exact current location
      turretMotor.setControl(motionMagicReq.withPosition(turretMotor.getPosition().getValueAsDouble()));
    }

    /**
 * Calculates the angle the turret needs to turn to face a field coordinate.
 */
  public void aimAtCoordinate(Translation2d target, Pose2d robotPose) {
    // 1. Calculate the difference in X and Y between the robot and the target
    double deltaX = target.getX() - robotPose.getX();
    double deltaY = target.getY() - robotPose.getY();

    // 2. Use atan2 to find the global angle to the target
    Rotation2d globalAngleToTarget = new Rotation2d(Math.atan2(deltaY, deltaX));

    // 3. Subtract the robot's current chassis heading 
    // (If the robot turns 90 degrees left, the turret must turn 90 degrees right to compensate)
    Rotation2d targetTurretAngle = globalAngleToTarget.minus(robotPose.getRotation());

    // 4. Send this angle to your Turret's PID Controller
    // Assuming you have a PID controller set up for your turret motor!
    double targetDegrees = targetTurretAngle.getDegrees();

    turretMotor.setControl(motionMagicReq.withPosition(targetDegrees / 360.0));
  }
}