// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final TalonFX climb = new TalonFX(ClimbConstants.CLIMB_ID, "Coco");

  public ClimbSubsystem() {
    configureHardware();
  }

  private void configureHardware(){
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.CurrentLimits.StatorCurrentLimit = 80.0;

    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.CurrentLimits.SupplyCurrentLimit = 40;

    climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // climbConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // climbConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.CLIMB_UP_ENCODER;
    // climbConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // climbConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.CLIMB_DOWN_ENCODER;

    climbConfig.Feedback.SensorToMechanismRatio = ClimbConstants.kClimbGearRatio;

    climb.getConfigurator().apply(climbConfig);

    climb.setPosition(0);
  }

  public void moveClimb(double speed){
    climb.set(speed);
  }

  public void stopClimb(){
    climb.stopMotor();
  }

  public double getClimbPosition() {
    return climb.getPosition().getValueAsDouble();
  }
}
