package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VortexConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftFly = new TalonFX(ShooterConstants.Left_Flywheel_Motor_ID, "Colby");
    private final TalonFX rightFly = new TalonFX(ShooterConstants.Right_Flywheel_Motor_ID, "Colby");
    private final TalonFX hood = new TalonFX(ShooterConstants.HOOD_ID, "Colby");

    private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0);
    private final VelocityTorqueCurrentFOC velocityRequestFOC = new VelocityTorqueCurrentFOC(0.0);
    
    // // add conditional in set rpm function to switch between bang bang and torque current depending on if up to speed or not
    // // Use torque current for when within tolerance
    // // Use bang bang for when getting up to speed and need to tune p for it (test in tunerx)
    // private final BangBangController bangBangController = new BangBangController();

    private final InterpolatingDoubleTreeMap distanceToRpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    
    // The Lookup Table: Keys are Distance (Meters), Values are ShooterStates
    private final InterpolatingTreeMap<Double, ShooterState> shooterMap = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(),
        // Custom interpolator to blend two ShooterStates together smoothly
        (start, end, q) -> new ShooterState(
            MathUtil.interpolate(start.rpm, end.rpm, q),
            MathUtil.interpolate(start.hoodAngle, end.hoodAngle, q),
            MathUtil.interpolate(start.timeOfFlight, end.timeOfFlight, q)
        )
    );

    private double currentTargetRPM = 0.0;
    private double currentTargetHood = 0;
    private boolean isSpunUp = false;   // Tracks if we reached speed to prevent stuttering

    public ShooterSubsystem() {
      configureHardware();

      // distance -> (RPM, Hood Angle, Time of Flight in seconds)
      shooterMap.put(1.23, new ShooterState(2575, 0.0, .71));
      shooterMap.put(2.46, new ShooterState(2800, 9.0, 1.02));  
      shooterMap.put(3.7, new ShooterState(3150, 15.0, 1.08));  
      shooterMap.put(4.93, new ShooterState(3700, 15.0, 1.34));
      shooterMap.put(5.644, new ShooterState(4175, 15.0, 1.5)); // Far Right Corner (furthest diagonal in launching zone)
      shooterMap.put(7.00, new ShooterState(5200, 15.0, 1.65)); 

      SmartDashboard.putNumber("Testing RPM", 3000);
      SmartDashboard.putNumber("Testing Hood Angle", 0);

      // distanceToRpmMap.put(1.23, 2700.0);
      // distanceToRpmMap.put(2.46, 3000.0);
      // distanceToRpmMap.put(3.7, 3750.0);
      // distanceToRpmMap.put(4.93, 4800.0);

      // hoodMap.put(1.23, 0.0);
      // hoodMap.put(2.46, 7.92);
      // hoodMap.put(3.7, 7.92);
      // hoodMap.put(4.93, 11.88);
    }

    private void configureHardware(){
      TalonFXConfiguration flyConfig = new TalonFXConfiguration();
      // Current Limits (crucial for launchers so we don't blow a breaker on spin-up)
      // Stator limit controls maximum torque/acceleration
      flyConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      flyConfig.CurrentLimits.StatorCurrentLimit = 80.0; // 80 Amps is (allegedly) incredibly punchy

      // Supply limit protects the battery and breaker
      flyConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      flyConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // The absolute max it will ever draw  
      flyConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0; // Drops to 40A if it pushes 60A  for too long
      flyConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5; // Wait half a second before dropping the limit
      
      // PID and Feedforward (Slot 0)
      flyConfig.Slot0.kS = ShooterConstants.kFlywheelS; // Static Friction (Amps)
      flyConfig.Slot0.kV = ShooterConstants.kFlywheelV; // Velocity Feedforward (Amps per RPS)
      flyConfig.Slot0.kP = ShooterConstants.kFlywheelP; // Proportional (Amps per error RPS)
      flyConfig.Slot0.kI = ShooterConstants.kFlywheelI;
      flyConfig.Slot0.kD = ShooterConstants.kFlywheelD;

      // Coast Mode - Never break a launchere 
      flyConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      flyConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // Apply to Leader
      leftFly.getConfigurator().apply(flyConfig);

      // Configure Follower
      rightFly.getConfigurator().apply(flyConfig);
      rightFly.setControl(new Follower(ShooterConstants.Left_Flywheel_Motor_ID, MotorAlignmentValue.Opposed));

      
      TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

      hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      hoodConfig.CurrentLimits.StatorCurrentLimit = 20.0; // Prevents the gearbox from self-destructing

      hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      hoodConfig.CurrentLimits.SupplyCurrentLimit = 20.0; // Protects the battery

      hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      hoodConfig.Feedback.RotorToSensorRatio = 1.0;
      hoodConfig.Feedback.SensorToMechanismRatio = 59.94;

      hoodConfig.Slot0.kP = ShooterConstants.kHoodkP;
      hoodConfig.Slot0.kI = ShooterConstants.kHoodkI;
      hoodConfig.Slot0.kD = ShooterConstants.kHoodkD;
      hoodConfig.Slot0.kV = ShooterConstants.kHoodkV;

      hoodConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kHoodCruiseVelocity;

      hoodConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.kHoodAcceleration;

      hoodConfig.MotionMagic.MotionMagicJerk = ShooterConstants.kHoodJerk;

      hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.06;
      hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.03;
      hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      hood.getConfigurator().apply(hoodConfig);
      hood.setPosition(0);
    }

    /**
     * Called by your Left Bumper command. Reads the map and spins up.
     */
    public void setTarget(double distanceMeters) {
      // FAILSAFE: Ignore blocked/lost Limelight distance updates
      if (distanceMeters < 0.5) return;
      
      ShooterState targetState = shooterMap.get(distanceMeters);

      currentTargetRPM = targetState.rpm;
      currentTargetHood = targetState.hoodAngle;
      // System.out.println("currentTargetHood"+currentTargetHood);

      setRPM(currentTargetRPM);
      setHoodAngle(currentTargetHood);
    }

    public void stop() {
      currentTargetRPM = 0.0;
      setRPM(0);
    }

    /**
     * Default command that keeps the flywheel spinning at a low idle RPM
     * whenever no scoring/ferry command is actively owning the shooter. 
     * 
     * This does NOT touch the hood - the hood is brake-mode and will hold its
     * last commanded position, which is what we want between shots. 
     * 
     * Because current limits are unchanged and the idle RPM is low, supply
     * draw at idle is small (typically under 10A total across both motors).
     */
    // public Command idleCommand() {
    //   return run(() -> {
    //     currentTargetRPM = kIdleRPM;
    //     setRPM(kIdleRPM);
    //   }).withName("ShooterIdle");
    // }

    /**
     * Gets the Time of Flight for the Turret's SOTM math
     */
    public double getTimeOfFlight(double distanceMeters) {
      distanceMeters = Math.max(distanceMeters, ShooterConstants.MAP_SHOOT_MIN);
      
      return shooterMap.get(distanceMeters).timeOfFlight;
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter RPM", getFlywheelRPM());
    }

    /**
     * The Gatekeeper check for the Fire Command
     */
    public boolean isReady(){
      double currentRpm = getFlywheelRPM();
      double flyError = Math.abs(currentRpm - currentTargetRPM);



      double currentHood = getHoodAngle();
      double hoodError = Math.abs(currentHood - (currentTargetHood/360));

      // Recovery logic: Moving shots (SOTM requires more energy)
      // tighten tolerance to 150 rpm (from 500 rpm) after the first shot to prevent "short" follow-up
      double tolerance = isSpunUp ? ShooterConstants.kRecoveryToleranceRPM : ShooterConstants.kToleranceRPM;

      boolean flywheelsAtSpeed = flyError < tolerance;

      isSpunUp = flywheelsAtSpeed;

      // System.out.println("hoodError:"+hoodError);

      boolean hoodAtAngle = hoodError < ShooterConstants.kHoodTolerance;

      // System.out.println("flywheelsAtSpeed"+flywheelsAtSpeed);
      // System.out.println("hoodAtAngle"+hoodAtAngle);

      return flywheelsAtSpeed && hoodAtAngle;
    }

    // --- Hardware methods go here ---
    public void setRPM(double rpm) {
      // Convert RPM to RPS for Phoenix 6
      double rps = rpm / 60.0;

      // Command BOTH motors with Torque Current FOC
      leftFly.setControl(velocityRequestFOC.withVelocity(rps));
    }

    // public void setRPMBangBang(double rpm) {
    //   // Convert RPM to RPS for Phoenix 6
    //   double rps = rpm / 60.0;

    //   // Command BOTH motors with Torque Current FOC
    //   if (isSpunUp) { // need to make isspunup update in a periodic function rather than only when isready is called (in firefuelcommand)
    //     leftFly.setControl(velocityRequestFOC.withVelocity(rps));
    //   }
    //   else {
    //     // no clue for syntax
    //     leftFly.bangBangController();
    //   }
    // }

    public void setHoodAngle(double degrees) {
      // double currentRotations = hood.getPosition().getValueAsDouble();
      // double gravityFF = (ShooterConstants.kHoodkG * Math.sin(currentRotations * 2 * Math.PI)) * (-1.0);
      // double targetRotations = degrees / 360;
      // hood.setControl(hoodPositionRequest.withPosition(targetRotations).withFeedForward(gravityFF));

      double targetRotations = degrees / 360.0;

      double currentPos = hood.getPosition().getValueAsDouble();
      double currentError = hood.getClosedLoopError().getValueAsDouble();
      double currentVoltage = hood.getMotorVoltage().getValueAsDouble();

      // SmartDashboard.putNumber("Hood/Target", targetRotations);
      // SmartDashboard.putNumber("Hood/Position", currentPos);
      // SmartDashboard.putNumber("Hood/Error", currentError);
      // SmartDashboard.putNumber("Hood/AppliedVolts", currentVoltage);

      hood.setControl(hoodPositionRequest.withPosition(targetRotations));
    }

    private double getFlywheelRPM() {
      return leftFly.getVelocity().getValueAsDouble() * 60;
    }

    private double getHoodAngle() {
      return hood.getPosition().getValueAsDouble();
    }

}