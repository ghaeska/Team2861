package frc.robot;

import java.security.Key;

/* CTRE Imports */
//import com.ctre.phoenix.motorcontrol.NeutralMode;

/* REV Robotics Imports */
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
    ** This year for 2026, we are going to be using NEO Vortex Motors instead of the Neo 1.1s.
    ** These motors have a much higher rpm when compared to the older style.
    **
    **                          NEO V1.1 Speed Table
    ** +---------+-------+-------------+--------------------------------+---------------+
    ** | Pinion: | Spur: | Gear Ratio: | Drive Free Speed (NEO, ft/sec) | ( NEO, m/sec) |
    ** +---------+-------+-------------+--------------------------------+---------------+
    ** |   12T   |  22T  |   5.50:1    |            13.51               |     4.12      | Low
    ** |   13T   |  22T  |   5.08:1    |            14.63               |     4.46      | Medium
    ** |   14T   |  22T  |   4.71:1    |            15.76               |     4.80      | High
    ** |   14T   |  21T  |   4.50:1    |            16.51               |     5.03      | Extra High 1
    ** |   14T   |  20T  |   4.29:1    |            17.34               |     5.28      | Extra High 2
    ** | 15T(16) |  20T  |   4.00:1    |            18.57               |     5.66      | Extra High 3
    ** |   16T   |  20T  |   3.75:1    |            19.81               |     6.04      | Extra High 4    
    ** |   16T   |  19T  |   3.56:1    |            20.86               |     6.36      | Extra High 5
    ** +---------+-------+-------------+--------------------------------+---------------+
    **
    **                          NEO Vortex Speed Table
    ** +---------+-------+-------------+--------------------------------+---------------+
    ** | Pinion: | Spur: | Gear Ratio: |    Drive Free Speed (ft/sec)   |   ( m/sec)    |
    ** +---------+-------+-------------+--------------------------------+---------------+
    ** |   12T   |  22T  |   5.50:1    |            16.15               |     4.92      | Low
    ** |   13T   |  22T  |   5.08:1    |            17.49               |     5.33      | Medium
    ** |   14T   |  22T  |   4.71:1    |            18.84               |     5.74      | High
    ** |   14T   |  21T  |   4.50:1    |            19.73               |     6.01      | Extra High 1
    ** |   14T   |  20T  |   4.29:1    |            20.72               |     6.32      | Extra High 2
    ** | 15T(16) |  20T  |   4.00:1    |            22.20               |     6.77      | Extra High 3
    ** |   16T   |  20T  |   3.75:1    |            23.68               |     7.22      | Extra High 4    
    ** |   16T   |  19T  |   3.56:1    |            24.93               |     7.60      | Extra High 5
    ** +---------+-------+-------------+--------------------------------+---------------+
    **
    ** As we can see from the tables above, the newer motor is around a 2.9 ft/sec gain.
    ** With each different set of gear combinations.  In 2025, we ran Medium gearing, in
    ** 2026, I'd like us to be running around Extra high 1 or 2.  I think we will start out
    ** with building our modules with the 14T/22T combo, which will result in the High gearing.
    ** Once the game is revealed, we can switch this up if we need to be faster and not care
    ** about being pushed around.
    **
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
    public static final double kWheelBase = Units.inchesToMeters(33.0);
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


    /* SPARK MAX CAN IDs */
    public static final int kFrontLeftDrivingCanId  = 27;
    public static final int kRearLeftDrivingCanId   = 25;
    public static final int kFrontRightDrivingCanId = 24;
    public static final int kRearRightDrivingCanId  = 21;

    public static final int kFrontLeftTurningCanId  = 3;
    public static final int kRearLeftTurningCanId   = 4;
    public static final int kFrontRightTurningCanId = 1;
    public static final int kRearRightTurningCanId  = 2;

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
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 0;
    public static final int k2ndDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants 
  {
    
  }

  public static final class NeoMotorConstants 
  {
    /* Free speed is from NEO Data Sheet REV-21-1650-DS */
    public static final double kFreeSpeedRpm = 5676;
  }


}

