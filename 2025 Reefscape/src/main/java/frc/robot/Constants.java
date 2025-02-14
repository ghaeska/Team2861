// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
public static final boolean DEBUG_MODE = true;

  public static class Field 
  {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);
  }

  public static final class DriveConstants 
  {
    /* PID control values for auto turning */
    public static final double k_turnPID_P = 0.00;
    public static final double k_turnPID_I = 0.00;
    public static final double k_turnPID_D = 0.00;
    public static final double k_turnPID_F = 0.00;

    public static final double k_tolerance_degrees = 2.0f;
  }

  public static final class AlgaeConstants
  {
    /* Algae Motor ID's */
    public static final int k_LeftAlgaeMotorCANId = 12;
    public static final int k_RightAlgaeMotorCANId = 13;

    public static final int k_Algae_MaxCurrent = 20;
  }

  public static final class CoralConstants
  {
    /* Coral Motor ID's */
    public static final int k_LeftCoralMotorCANId = 15;
    public static final int k_RightCoralMotorCANId = 16;
    public static final int k_PivotCoralMotorCANId = 17;

    public static final int k_Coral_MaxCurrent = 40;

    /* Coral Beam Break Sensors */
    public static final int k_DIO_LeftCoralSensorID = 1;
    public static final int k_DIO_RightCoralSensorID = 2;

    /* Because the Coral is on a pivot, we need to have a PID to hold the position */
    public static final double k_PivotCoralMotorP  = 0.006;
    public static final double k_PivotCoralMotorI  = 0.00; 
    public static final double k_PivotCoralMotorD  = 0.00;
    public static final double k_PivotCoralMotorFF = 0.00;

    public static final double k_PivotMinOutput = -0.5;
    public static final double k_PivotMaxOutput = 0.5;

    /* Absolute Encoder for knowing exact angle */
    public static final int k_PivotCoralEncoderId = 0;

    /* Absolute Encoder Offset */
    public static final double k_PivotCoralEncoderOffset = 0.000000;

    /* Pivot Angle Set points */
    public static final Rotation2d k_PivotCoralAngleStowed = Rotation2d.fromDegrees( 210 );
    public static final Rotation2d k_PivotCoralAngleL1 = Rotation2d.fromDegrees( 115 );
    public static final Rotation2d k_PivotCoralAngleL2 = Rotation2d.fromDegrees( 115 );
    public static final Rotation2d k_PivotCoralAngleL3 = Rotation2d.fromDegrees( 115);
    public static final Rotation2d k_PivotCoralAngleL4 = Rotation2d.fromDegrees( 70 );
    public static final Rotation2d k_PivotCoralAngleSource = Rotation2d.fromDegrees(180);
    public static final Rotation2d k_PivotCoralAngleMax = Rotation2d.fromDegrees( 220 );
    public static final Rotation2d k_PivotCoralAngleMin = Rotation2d.fromDegrees( 0 );

  }

  public static final class ElevatorConstants
  {
    /* Elevator Motor ID's */
    public static final int k_LeftElevatorMotorCANId = 10;
    public static final int k_RightElevatorMotorCANId = 11;

    /* Elevator PID Settings */
    public static final double k_Ele_PID_P = 0.012;
    public static final double k_Ele_PID_I = 0;
    public static final double k_Ele_PID_D = 0.0;
    public static final double k_Ele_PID_FF = 0.0;

    public static final double k_Ele_MaxVelocity = 65;
    public static final double k_Ele_MaxAcceleration = 200;

    public static final int k_Ele_MaxCurrent = 40;

    public static final double k_Ele_StowHeight = 0.0;
    public static final double k_Ele_SrcHeight = 8.75;    
    public static final double k_Ele_L1Height = 0;
    public static final double k_Ele_L2Height = 38;
    public static final double k_Ele_L3Height = 62;
    public static final double k_Ele_L4Height = 80;
    public static final double k_Ele_MaxHeight = 84;
    //public static final double k_Ele_GroundAlgaeHeight = 0.0;
    public static final double k_Ele_ScoreAlgaeHeight = 0.0;
    //public static final double k_Ele_LowAlgaeHeight = 24.8;
    public static final double k_Ele_HighAlgaeHeight = 40;

  }

  public static final class AutoConstants 
  {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double k_tolerance_degrees = 2.0f;
  }

}
