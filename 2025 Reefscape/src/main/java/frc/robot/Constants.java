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

    public static final int k_Algae_MaxCurrent = 30;
  }

  public static final class CoralConstants
  {
    /* Coral Motor ID's */
    public static final int k_LeftCoralMotorCANId = 15;
    public static final int k_RightCoralMotorCANId = 16;
    public static final int k_PivotCoralMotorCANId = 17;

    public static final int k_Coral_MaxCurrent = 30;

    /* Coral Beam Break Sensors */
    public static final int k_DIO_LeftCoralSensorID = 1;
    public static final int k_DIO_RightCoralSensorID = 2;

    /* Because the Coral is on a pivot, we need to have a PID to hold the position */
    public static final double k_PivotCoralMotorP  = 0.026;
    public static final double k_PivotCoralMotorI  = 0.00; 
    public static final double k_PivotCoralMotorD  = 0.00;
    public static final double k_PivotCoralMotorFF = 0.001;

    public static final double k_PivotMinOutput = -0.5;
    public static final double k_PivotMaxOutput = 0.5;

    /* Absolute Encoder for knowing exact angle */
    public static final int k_PivotCoralEncoderId = 0;

    /* Absolute Encoder Offset */
    public static final double k_PivotCoralEncoderOffset = 0.000000;

    /* Pivot Angle Set points */
    public static final Rotation2d k_PivotCoralAngleStowed = Rotation2d.fromDegrees( 0 );
    public static final Rotation2d k_PivotCoralAngleL1 = Rotation2d.fromDegrees( 0 );
    public static final Rotation2d k_PivotCoralAngleL2 = Rotation2d.fromDegrees( 0 );
    public static final Rotation2d k_PivotCoralAngleL3 = Rotation2d.fromDegrees( 0 );
    public static final Rotation2d k_PivotCoralAngleL4 = Rotation2d.fromDegrees( 0 );


  }

  public static final class ElevatorConstants
  {
    /* Elevator Motor ID's */
    public static final int k_LeftElevatorMotorCANId = 10;
    public static final int k_RightElevatorMotorCANId = 11;

    /* Elevator PID Settings */
    public static final double k_Ele_PID_P = 0.027;
    public static final double k_Ele_PID_I = 0;
    public static final double k_Ele_PID_D = 0.0;
    public static final double k_Ele_PID_FF = 0.0085;
    public static final double k_Ele_PID_IZone = 5.0;
    public static final double k_Ele_PID_G = 0.5;

    public static final double k_Ele_MaxVelocity = 65;
    public static final double k_Ele_MaxAcceleration = 200;

    public static final int k_Ele_MaxCurrent = 40;

    public static final double k_Ele_StowHeight = 0.0;
    public static final double k_Ele_SrcHeight = 8.75;    
    public static final double k_Ele_L1Height = 3.0;
    public static final double k_Ele_L2Height = 5.5;
    public static final double k_Ele_L3Height = 21.5;
    public static final double k_Ele_L4Height = 52.5;
    public static final double k_Ele_MaxHeight = 56.2;
    //public static final double k_Ele_GroundAlgaeHeight = 0.0;
    public static final double k_Ele_ScoreAlgaeHeight = 0.0;
    //public static final double k_Ele_LowAlgaeHeight = 24.8;
    public static final double k_Ele_HighAlgaeHeight = 40;

  }

  

  // public static class Arm
  // {
  //   public static final double k_ArmGearRatio = ( 1/100 ) * ( 24/72 ); // GTH:TODO need to update with proper values. (24/54)
  //   public static final double k_ArmPositionFactor = k_ArmGearRatio * 2.0 * Math.PI;
  //   public static final double k_ArmVelocityFactor = k_ArmGearRatio * 2.0 * Math.PI / 60.0;
  //   public static final double k_ArmFreeSpeed = 5676.0 * k_ArmVelocityFactor;
  //   public static final double k_ArmZeroCosineOffset = 0.873; //GTH:TODO need to get proper angle and convert to raidans, currently at 50degrees

  //   public static final TrapezoidProfile.Constraints k_ArmMotionConstraint = new TrapezoidProfile.Constraints(1.0, 2.0);
  //   public static final ArmFeedforward k_ArmFeedForward = new ArmFeedforward( 0.5, 0.25, 3.45, 0.01 );//values taken from team 3467

  //   /* Arm Motor ID's */
  //   public static final int k_ArmLeftMotorCanId = 15;
  //   public static final int k_ArmRightMotorCanId = 16;

  //   public static final double k_ArmMinOutput = -0.5;
  //   public static final double k_ArmMaxOutput =  0.5; //GTH:TODO need to update value
   
  //   /* Arm PID constants */
  //   public static final double k_ArmMotorP  = 0.026;
  //   public static final double k_ArmMotorI  = 0.00; 
  //   public static final double k_ArmMotorD  = 0.00;
  //   public static final double k_ArmMotorFF = 0.001;
  //   public static final double k_ArmCruise  = 4.0;
  //   public static final double k_ArmAccel   = 10;

  //   /* Digital Input/Output ID's */
  //   public static final int k_ArmEncoderId = 0;

  //   /* Absolute Encoder Offset */
  //   public static final double k_ArmEncoderOffset = 0.000000; //GTH:TODO need to get value

  //   /* Pivot Angle Set Points */
  //   //public static final double k_ArmAngleSource = 190; //GTH:TODO need to get value
  //   public static final Rotation2d k_ArmAngleAmp    = Rotation2d.fromDegrees(160); //GTH:TODO need to update values every time chain skips
  //   public static final Rotation2d k_ArmAngleStowed = Rotation2d.fromDegrees(222); //GTH:TODO need to update values every time chain skips
  //   //public static final double k_ArmAngleSpeaker = 270; //GTH:TODO need to get value
  //   //public static final double k_ArmAngleStage = 270; //GTH:TODO need to get value
  //   //public static final double k_ArmAnglePass = 270; //GTH:TODO need to get value

  // }

  // public static class Index
  // {
  //   /* Index Motor ID's */
  //   public static final int k_IndexMotorCanId = 11;

  //   /* Index Beam Break Digital Input ID */
  //   public static final int k_DIO_IndexSensorID = 1;
    
  //   /* Index Motor Speeds */
  //   public static final double k_IndexForwardSpeed =  0.3;
  //   public static final double k_IndexReverseSpeed = -0.3;
  // }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;
  public static final double kDriveDeadband = 0.05;
  public static final double k_tolerance_degrees = 2.0f;
}



}
