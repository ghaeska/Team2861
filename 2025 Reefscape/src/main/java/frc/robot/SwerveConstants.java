package frc.robot;

import java.security.Key;

/* CTRE Imports */
//import com.ctre.phoenix.motorcontrol.NeutralMode;

/* REV Robotics Imports */
//import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/* WPI Library Imports */
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import frc.robot.Constants.NeoMotorConstants;


public final class SwerveConstants 
{
  public static final class DriveConstants 
  {
    /*
    ** Driving Parameters - Note that these are not the maximum capable speeds of
    ** the robot, rather the allowed maximum speeds
    **
    ** +---------+-------------+--------------------------------+---------------+
    ** | Pinion: | Gear Ratio: | Drive Free Speed (NEO, ft/sec) | ( NEO, m/sec) |
    ** +---------+-------------+--------------------------------+---------------+
    ** |   12T   |   5.50:1    |            13.51               |     4.11      |
    ** |   13T   |   5.08:1    |            14.63               |     4.45      |
    ** |   14T   |   4.71:1    |            15.76               |     4.80      |
    ** +---------+-------------+--------------------------------+---------------+
    */
    public static final double kMaxSpeedMetersPerSecond = 4.45;

    /* Angular speed, I believe the lower the number, the slower the turn. */
    public static final double kMaxAngularSpeed = 1.0 * Math.PI; // radians per second

    /* Slew rates slow down the initial input from a controller and will ramp up over time. */
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(  kWheelBase / 2,  kTrackWidth / 2 ),
        new Translation2d(  kWheelBase / 2, -kTrackWidth / 2 ),
        new Translation2d( -kWheelBase / 2,  kTrackWidth / 2 ),
        new Translation2d( -kWheelBase / 2, -kTrackWidth / 2 ) );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset  = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset =  0;
    public static final double kBackLeftChassisAngularOffset   =  Math.PI;
    public static final double kBackRightChassisAngularOffset  =  Math.PI / 2;

    /*
    ** Front Left:
    ** Turning Motor CAN ID: 1
    ** Driving Motor CAN ID: 2
    ** Front Right:
    ** Turning Motor CAN ID: 3
    ** Driving Motor CAN ID: 4
    ** Back Left:
    ** Turning Motor CAN ID: 5
    ** Driving Motor CAN ID: 6
    ** Back Right:
    ** Turning Motor CAN ID: 7
    ** Driving Motor CAN ID: 8
    */

    /* SPARK MAX CAN IDs */
    public static final int kFrontLeftDrivingCanId  = 2;
    public static final int kRearLeftDrivingCanId   = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId  = 8;

    public static final int kFrontLeftTurningCanId  = 1;
    public static final int kRearLeftTurningCanId   = 5;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId  = 7;

    /* Gyro CAN ID */
    public static final boolean kGyroReversed       = true;
    public static final int kGyroCanId              = 20;
  }

  public static final class ModuleConstants 
  {
    /* Constants for Feedforward from SYSID */
    public static final double kS = 0.034;
    public static final double kV = 0.3;
    public static final double kA = 0.018;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

    
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    /* These are moved to Configs.java */
    // public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
    //     / kDrivingMotorReduction; // meters
    // public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
    //     / kDrivingMotorReduction) / 60.0; // meters per second

    /* These are moved to Configs.java */
    // public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    // public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    /* These are moved to Configs.java */
    // public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    // public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    /* These are moved to Configs.java */
    // public static final double kDrivingP = 0.04;
    // public static final double kDrivingI = 0;
    // public static final double kDrivingD = 0;
    // public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    // public static final double kDrivingMinOutput = -1;
    // public static final double kDrivingMaxOutput = 1;

    /* These are moved to Configs.java */
    // public static final double kTurningP = 1;
    // public static final double kTurningI = 0;
    // public static final double kTurningD = 0;
    // public static final double kTurningFF = 0;
    // public static final double kTurningMinOutput = -1;
    // public static final double kTurningMaxOutput = 1;

    /* These are moved to Configs.java */
    // public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    // public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    /* These are moved to Configs.java */
    // public static final int kDrivingMotorCurrentLimit = 40; // amps
    // public static final int kTurningMotorCurrentLimit = 20; // amps  
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 0;
    public static final int k2ndDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants 
  {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants 
  {
    /* Free speed is from NEO Data Sheet REV-21-1650-DS */
    public static final double kFreeSpeedRpm = 5676;
  }
}
