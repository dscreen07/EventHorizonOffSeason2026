package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterTestingCommand extends Command {
    private final ShooterSubsystem shooter;

    public RevShooterTestingCommand(ShooterSubsystem s) {
        this.shooter = s;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double testRPM = SmartDashboard.getNumber("Testing RPM", 0.0);
        double testAngle = SmartDashboard.getNumber("Testing Hood Angle", 0.0);
        shooter.setRPM(testRPM);
        shooter.setHoodAngle(testAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}