package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;

    public RevShooterCommand(CommandSwerveDrivetrain dt, ShooterSubsystem s) {
        this.drivetrain = dt;
        this.shooter = s;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d target = new Translation2d();
        
        boolean isOffensive = (alliance == Alliance.Blue) 
            ? pose.getX() < ShooterConstants.BLUE_CENTERLINE_X 
            : pose.getX() > ShooterConstants.RED_CENTERLINE_X;

        if (isOffensive) {
            // sets target to hub
            target = (alliance == Alliance.Blue) ? ShooterConstants.BLUE_HUB : ShooterConstants.RED_HUB;
        } else {
            double y_coord = pose.getY();
            if (alliance == Alliance.Blue) {
                    target = (y_coord > ShooterConstants.CENTERLINE_Y) ? ShooterConstants.BLUE_PASS_ZONE_LEFT : ShooterConstants.BLUE_PASS_ZONE_RIGHT;
                }
            else {
                    target = (y_coord > ShooterConstants.CENTERLINE_Y) ? ShooterConstants.RED_PASS_ZONE_RIGHT : ShooterConstants.RED_PASS_ZONE_LEFT;
                }


        }
        // ===== NEW SOTM FIX =====

        // Get the raw physical distance
        double baseDistance = pose.getTranslation().getDistance(target);

        // Get the field-relative speeds (Properly rotated)
        var speeds = drivetrain.getState().Speeds;
        Translation2d fieldVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                                            .rotateBy(pose.getRotation()) ;

        // Calculate a safe Time of Flight
        double flightTime = shooter.getTimeOfFlight(baseDistance);

        // Calculate the Virtual Target to compensate for chassis speed
        Translation2d virtualTarget = new Translation2d(
            target.getX() - (fieldVelocity.getX() * flightTime),
            target.getY() - (fieldVelocity.getY() * flightTime)
        );

        // Calculate the new SOTM distance
        double virtualDistance = pose.getTranslation().getDistance(virtualTarget);

        // Safety Clamp: Prevent the map from returning 0 RPM
        virtualDistance = Math.min(virtualDistance, ShooterConstants.MAP_SHOOT_MAX);

        // Spool the Vortex and Wormhole
        shooter.setTarget(virtualDistance);

        // ===== END OF SOTM FIX =====


        // double distance = pose.getTranslation().getDistance(target);
        // // note since using interpolation for passing all passing shots will reach the spot but 6ish feet in the air
        // shooter.setTarget(distance);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}