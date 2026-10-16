// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClimbCommand extends Command {
  /** Creates a new RunClimbCommand. */
  private final ClimbSubsystem climb;
  private final double speed;
  private final double position;

  public RunClimbCommand(ClimbSubsystem climb, double speed, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.speed = speed;
    this.position = position;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(climb.getClimbPosition() - position) > ClimbConstants.kClimbTolerance) {
      climb.moveClimb(speed);
    }
    else {
      climb.stopClimb();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
