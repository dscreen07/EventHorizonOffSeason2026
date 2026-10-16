package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 *  CONSTANTS CLASS
 *  ----------------
 *  This class holds all the "Magic Numbers" for the robot.
 *  By keeping them here, we can change a Motor ID or a PID value in one place
 *  without hunting through the entire codebase.
 */

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;


public class Constants {

  public static final class TurretConstants{
    public static final int TURRET_ID = 20;
    public static final double TURRET_SPEED = 0.5;

    // Fused CAN requires the exact gear ratio between the motor and CANCoder
    public static final double kTurretGearRatio = 96.0;


    // IMPORTANT: Measure these by hand!
    // Rotate the turret to far left, check "Position (Rot)" in Elastic, put that here.
    public static final double kForwardLimit = 0.5;
    public static final double kReverseLimit = -0.5;

    // PID Gains (for later)
    public static final double kTurretkP = 90.0;
    public static final double kTurretkI = 0.0;
    public static final double kTurretkD = 2.5;
    public static final double kTurretkV = 6.5; 
    public static final double kTurretkS = 0.42; 

    public static final double kTurretCruiseVelocity = 0.9; // previously 0.8 but turret wasn't keeping up with chassis
    public static final double kTurretAcceleration = 40.0;  // previously 4.0 but turret wasn't keeping up with chassis
    public static final double kTurretJerk = 400.0;

    public static final double TURRET_TOLERANCE = 2.0;

  }

  public static final class ShooterConstants {
    public static final double RED_CENTERLINE_X = 11.95;
    public static final double BLUE_CENTERLINE_X = 4.62;
    public static final double CENTERLINE_Y = 4.035;

    public static final double MAP_SHOOT_MAX = 7.00;
    public static final double MAP_SHOOT_MIN = 1.23;

    public static final Translation2d BLUE_HUB = new Translation2d(4.612, 4.021);
    public static final Translation2d RED_HUB = new Translation2d(11.901, 4.021);
    public static final Translation2d BLUE_PASS_ZONE_LEFT = new Translation2d(1.088, 6.994);
    public static final Translation2d BLUE_PASS_ZONE_RIGHT = new Translation2d(1.088, 1.216);
    public static final Translation2d RED_PASS_ZONE_LEFT = new Translation2d(15.452, 1.216);
    public static final Translation2d RED_PASS_ZONE_RIGHT = new Translation2d(15.452, 6.994);

    // CAN IDs
    public static final int Left_Flywheel_Motor_ID = 21;
    public static final int Right_Flywheel_Motor_ID = 22;
    public static final int HOOD_ID = 24;

    public static final double kMaxRpm = 6000.0;
    public static final double HOOD_SPEED = 0.03;

    // *** Amps PID ***
    // kS: How many Amps does it take just to overcome static friction and barely spin? 
    public static final double kFlywheelS = 0;
    // kV: How many Amps does it take to maintain 1 rotation Per Second (RPS)?
    public static final double kFlywheelV = 0.13; //0.07;
    // kP: How aggressively (in Amps) should we push if we are 1 RPS off target?
    public static final double kFlywheelP = 10.0; //1.75;
    public static final double kFlywheelI = 0.0; // Leave at 0 for launchers
    public static final double kFlywheelD = 0.0; // Usually 0 for launchers

    // --- HOOD PID & FF ---
    public static final double kHoodkV = 0.18;              //  tuned
    public static final double kHoodkP = 80.0; //30;               //  tuned
    public static final double kHoodkI =  5.0; // 0.0;               //  tuned
    public static final double kHoodkD = 0.0;               //  tuned
    public static final double kHoodkG = 0.23;               // tuned
    public static final double kHoodkS = 0.25;

    public static double kHoodCruiseVelocity = 3;
    public static double kHoodAcceleration = 20;
    public static double kHoodJerk = 0.0;

    // --- SAFETY ---
    public static final double kHoodMinAngle = 0.0;         // Rotations
    public static final double kHoodMaxAngle = 0.48;       
    public static final int kFlywheelCurrentLimit = 80;       // High current for spin-up
    public static final double kToleranceRPM = 150.0;         // Intial spin-up tolerance
    public static final double kRecoveryToleranceRPM = 500.0; // Tolerance for follow-up shots after initial spin-up
    public static final double kHoodGearRatio = 29.968;
    public static final double kHoodTolerance = 0.005;

  }

  public static final class WormholeConstants {
    public static final int WORMHOLE_ID = 23;
    public static final double WORMHOLE_SPEED = 0.95;
  }

  public static final class VortexConstants {
    public static final int VORTEX_ID = 30;
    public static final double VORTEX_SPEED = 0.95;

    // Constants for Vortex Jamming
    public static final double JAM_CURRENT_THRESHOLD = 20.0;    // Amps - Tuned/Tested 30 before
    public static final double UNJAM_DURATION_SECONDS = 0.5;    // How long to reverse for
    public static final double VORTEX_REVERSE_SPEED = -0.5;     // Speed to run when reversing   
  }

  public static final class IntakeConstants{
    // --- CAN IDs ---
    public static final int INTAKE_HINGE_LEFT_ID = 31;      // Kraken X44
    public static final int INTAKE_HINGE_RIGHT_ID = 32;     // Kraken X44
    public static final int ROLLER_ID = 33;                 // Kraken X60

    // --- FourBar Positions (Rotations) ---
    // 60:1 total reduction for the 4-bar arm
    public static final double kHingeGearRatio = 60.0;
    // 0.0 = Vertical (Stowed), 0.25 = Horizontal (Deployed, 90 deg)
    // ADJUST based on Supernova's specific mechanism zero-point!
    public static final double kDeployPosition = 0.233;       
    public static final double kStowPosition = 0.0;        

    // --- CONSTRAINTS ---
    public static final double kForwardSoftLimit = 0.0;    //  was 0.02 Just past Stowed
    public static final double kReverseSoftLimit = -0.0;    // was -0.27  Just past Deployed

    // --- PID Gains (Tune these!) ---
    // Start with P low. Kraken is very strong
    public static final double kHingekG = 0.5;              // tuned with mdf version
    public static final double kHingekV = 7.2;              // tuned with mdf version
    public static final double kHingekP = 12.0;             // tuned with mdf version
    public static final double kHingekI = 0.0;              // tuned with mdf version
    public static final double kHingekD = 0.0;              // tuned with mdf version

    // Supply Current Limit (Amps) - Prevent burning the motor if jammed
    public static final int kHingeCurrentLimit = 40;
    public static final int kRollerCurrentLimit = 60; 

    public static double kHingeCruiseVelocity = 0.5;
    public static double kHingeAcceleration = 1.0;
    public static double kHingeJerk = 0.0;

    // --- ROLLER ---
    // Bringing the Fuel In
    public static final double kRollerIntakeSpeed = -0.95;
    public static final double kRollerHoldSpeed = -.20;
    // Pusing Fuel out
    public static final double kRollerOuttakeSpeed = 0.50;
    // Stop
    public static final double kRollerStopSpeed = 0.0;
  }

  public static final class ClimbConstants {
    public static final int CLIMB_ID = 34;
    public static final double CLIMB_SPEED = 0.75;
    public static final double kClimbTolerance = 0.05; // adjust based on how we do sensor to mechanism ratio
    public static final double CLIMB_UP_ENCODER = 200; // adjust based on how we do sensor to mechanism ratio
    public static final double CLIMB_DOWN_ENCODER = 0;
    public static final double kClimbGearRatio = 1;
  }

  public static final class VisionConstants {
    // Translation (Meters)
    public static final double kCameraForward = 0.0400304;  // Positive is forward of robot center (1.576" forward from center)
    public static final double kCameraSide = -0.244475;       // Positive is RIGHT of robot center (9.625" left of center)
    public static final double kCameraUp = 0.5108448;        // Distance from floor to lens (20.112")
    
    // Rotation (Degrees)
    public static final double kCameraRoll = 0.0;     // Mounted horizontally
    public static final double kCameraPitch = 25.0;   // Tilted up is positive
    public static final double kCameraYaw = 0.0;      // Pointing straight forward
  }

   
}
