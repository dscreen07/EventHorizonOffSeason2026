package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.lang.Math;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX hingeLead = new TalonFX(IntakeConstants.INTAKE_HINGE_RIGHT_ID);
    private final TalonFX hingeFollower = new TalonFX(IntakeConstants.INTAKE_HINGE_LEFT_ID);
    private final TalonFX rollers = new TalonFX(IntakeConstants.ROLLER_ID);

    // Reuse the request object to save RAM
    private final MotionMagicVoltage hingeMMRequest = new MotionMagicVoltage(0).withSlot(0);

    public IntakeSubsystem(){
       configureHardware();
    }

    private void configureHardware(){
        
        // Create a clean config object
        TalonFXConfiguration hingeConfig = new TalonFXConfiguration();
        // hingeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hingeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kHingeGearRatio;

        // Setup PID (Slot 0)
        hingeConfig.Slot0.kP = IntakeConstants.kHingekP;
        hingeConfig.Slot0.kI = IntakeConstants.kHingekI;
        hingeConfig.Slot0.kD = IntakeConstants.kHingekD;
        hingeConfig.Slot0.kV = IntakeConstants.kHingekV;    // Helps the arm move faster

        // Current Limits (Protect the Motor)
        hingeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kHingeCurrentLimit;
        hingeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hingeConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kHingeCruiseVelocity;

        hingeConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.kHingeAcceleration;

        hingeConfig.MotionMagic.MotionMagicJerk = IntakeConstants.kHingeJerk;

        // Apply Configs
        hingeLead.getConfigurator().apply(hingeConfig);
        hingeFollower.getConfigurator().apply(hingeConfig);

        // Follower Sync
        hingeFollower.setControl(new Follower(IntakeConstants.INTAKE_HINGE_RIGHT_ID, MotorAlignmentValue.Opposed));

        // Set Neutral Mode (Brake is safer for arms)
        hingeLead.setNeutralMode(NeutralModeValue.Brake);
        hingeFollower.setNeutralMode(NeutralModeValue.Brake);

        // Zero the sensor (Assume robot starts with intake STOWED)
        hingeLead.setPosition(0);
        hingeFollower.setPosition(0);

        TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
        rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kRollerCurrentLimit;

        rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollersConfig.CurrentLimits.SupplyCurrentLimit = 20.0; // 25 before

        rollers.getConfigurator().apply(rollersConfig);
    }

    public void setHingePositionMM(double targetRotations) {
        // Current angle (for gravity feedforward)
        double currentRotations = hingeLead.getPosition().getValueAsDouble();
        double angleRadians = currentRotations * 2 * Math.PI;

        // Gravity feedforward
        double gravityFF = (IntakeConstants.kHingekG * Math.sin(angleRadians))*(-1.0);

        // Motion Magic position command
        hingeLead.setControl(hingeMMRequest.withPosition(targetRotations).withFeedForward(gravityFF));
    }

    /**
     * Spins the intake rollers.
     * @param percentOutput Speed from -1.0 to 1.0
     */
    public void setRollerSpeed(double percentOutput){
        rollers.set(percentOutput);
    }

    public void stopRollers(){
        rollers.stopMotor();;
    }

    public void setHingeOutput(double percent){
        hingeLead.set(percent);
    }

    public void setRollerOutput(double percent){
        rollers.set(percent);
    }

    public void stop(){
        hingeLead.stopMotor();
        rollers.stopMotor();
    }

    public TalonFX getLeader(){
        return hingeLead;
    }

    public double getEncoder() {
        return hingeLead.getPosition().refresh().getValueAsDouble();
    }

    public double getVelocity() {
        return hingeLead.getVelocity().refresh().getValueAsDouble();
    }

    public double getError() {
        return hingeLead.getClosedLoopError().refresh().getValueAsDouble();
    }

    public double getHingeRotations(){
        return hingeLead.getPosition().getValueAsDouble();
    }
       

}