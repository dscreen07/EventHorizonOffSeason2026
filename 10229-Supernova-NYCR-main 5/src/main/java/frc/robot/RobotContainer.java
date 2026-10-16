// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.AutoDeployIntakeCommand;
import frc.robot.commands.AutoTurretPassiveTrackCommand;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FireFuelCommand;
import frc.robot.commands.RevShooterCommand;
import frc.robot.commands.RunClimbCommand;
import frc.robot.commands.StowIntakeCommand;
import frc.robot.commands.TurretPassiveTrackCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VortexSubsystem;
import frc.robot.subsystems.WormholeSubsystem;

// PATHPLANNER IMPORTS

public class RobotContainer{

    // ================================================
    // DRIVETRAIN INITIALIZATION (MUST BE FIRST)
    // ================================================
    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(maxSpeed);


    // ======================================================
    // SUBSYSTEM INITIALIZATION
    // ======================================================
    private final TurretSubsystem turret = new TurretSubsystem(drivetrain);
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final WormholeSubsystem wormhole = new WormholeSubsystem();
    private final VortexSubsystem vortex = new VortexSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final VisionSubsystem limelight = new VisionSubsystem(drivetrain);
 

    // CONTROLLERS
    private final CommandXboxController driver_joystick = new CommandXboxController(0);
    private final CommandXboxController gunner_joystick = new CommandXboxController(1);


    // AUTO CHOOSER
    private final SendableChooser<Command> autoChooser; 


    public RobotContainer(){
        // ====================================================
        // PATHPLANNER NAMED COMMANDS TO GO HERE
        // ====================================================
        NamedCommands.registerCommand("StowIntake", new StowIntakeCommand(intake));
        NamedCommands.registerCommand("DeployIntake", new AutoDeployIntakeCommand(intake));
        NamedCommands.registerCommand("TurretPassiveTrackCommand", new AutoTurretPassiveTrackCommand(drivetrain, turret, shooter));
        NamedCommands.registerCommand("RevShooterCommand", new RevShooterCommand(drivetrain, shooter));
        NamedCommands.registerCommand("FireFuelCommand", new FireFuelCommand(shooter, turret, wormhole, vortex));


        RobotController.setBrownoutVoltage(Voltage.ofBaseUnits(6, Volts));

        // BUILD AUTO CHOOSER
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
        configureDefaultCommands();
    }



  private void configureBindings() {
    // Pressing X just revs the shooter (good for pre-spinning while driving or testing)
//    gunner_joystick.x().whileTrue(new RunFeederCommand(vortex, wormhole));

    // gunner_joystick.x().whileTrue(new FireFuelCommand(shooter, turret, wormhole, vortex));
    driver_joystick.leftTrigger(0.5).whileTrue(
        new RevShooterCommand(drivetrain, shooter)
        .alongWith(new FireFuelCommand(shooter, turret, wormhole, vortex)));
   
    driver_joystick.rightTrigger(0.5).whileTrue(new DeployIntakeCommand(intake, () -> driver_joystick.b().getAsBoolean()));

    // Climb
    driver_joystick.povUp().whileTrue(new RunClimbCommand(climb, ClimbConstants.CLIMB_SPEED, ClimbConstants.CLIMB_UP_ENCODER));
    driver_joystick.povDown().whileTrue(new RunClimbCommand(climb, (-ClimbConstants.CLIMB_SPEED), ClimbConstants.CLIMB_DOWN_ENCODER));

     // Pressing Y runs BOTH commands simultaneously
    // It will rev the shooter, and the moment it hits the target RPM, it fires.
    driver_joystick.y().whileTrue(new StowIntakeCommand(intake));
    

    // Driver Controls
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-driver_joystick.getLeftY() * maxSpeed) 
                .withVelocityY(-driver_joystick.getLeftX() * maxSpeed) 
                .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate) 
        )
    );

    // Slow Mode Override (Triggered by Left Trigger)
    driver_joystick.leftBumper().whileTrue(
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(-driver_joystick.getLeftY() * maxSpeed * 0.2)
                  .withVelocityY(-driver_joystick.getLeftX() * maxSpeed * 0.2)
                  .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate * 0.3)
        )
    );

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driver_joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

    driver_joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }


  private void configureDefaultCommands() {
    // Turret: passively track the HUB / pass zone based on pose 
    turret.setDefaultCommand((new TurretPassiveTrackCommand(
        drivetrain, turret, shooter, 
        () -> gunner_joystick.getRightX(), 
        () -> gunner_joystick.button(10).getAsBoolean())));

    // Shooter: idle the flywheel at a low RPM between shots. 
    //
    // This keeps kinetic enery in the wheel so spin-up to scoring/ferry RPM
    // is fast AND cheap on the battery. When RevShootCommand (or any other
    // command that requires the shooter) is scheduled, it interrupts this 
    // default; when that command ends, this default automatically resumes. 
   // shooter.setDefaultCommand(shooter.idleCommand());
    
  }

  public Command getAutonomousCommand() {
      // Returns the PathPlanner auto selected on the SmartDashboard
      return autoChooser.getSelected();
  }

}