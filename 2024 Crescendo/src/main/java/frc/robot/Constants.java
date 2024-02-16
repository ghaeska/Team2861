// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
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

  public static class Intake
  {
    /* Intake Motor ID's */
    public static final int k_IntakeMotorCanId = 10;  

    /* Intake Speeds */
    /* positive value ejects, negative intakes. */
    public static final double k_IntakeIntakeSpeedSlow = 0.5; 
    public static final double k_IntakeIntakeSpeedFast = 1.0; 
    public static final double k_IntakeEjectSpeed = -0.3;
  }

  public static class Shooter
  {
    /* Shooter PID constants */
    public static final double k_ShooterMotorP = 0.000005;
    public static final double k_ShooterMotorI = 0.0;
    public static final double k_ShooterMotorD = 0.0;
    public static final double k_ShooterMotorFF = 0.00002;

    /* Shooter Motor ID's */
    public static final int k_ShooterTopMotorCanId = 12;
    public static final int k_ShooterBotMotorCanId = 13;

    /* Shooter Min/Max Outputs */
    public static final double k_ShooterMinOutput = 0;
    public static final double k_ShooterMaxOutput = 1;
    
    /* Shooter Speeds (RPM) */
    public static final double k_ShooterSpeed_Speaker = 2500; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Amp = 1000; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Stage = 4000; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Pass = 1000; //GTH:TODO need to update value

    // public static final double k_ShooterSpeed_Speaker = 0.4; //GTH:TODO need to update value
    // public static final double k_ShooterSpeed_Amp = 0.4; //GTH:TODO need to update value
    // public static final double k_ShooterSpeed_Stage = 0.4; //GTH:TODO need to update value
    // public static final double k_ShooterSpeed_Pass = 0.4; //GTH:TODO need to update value
  }

  public static class Arm
  {
    public static final double k_ArmGearRatio = ( 1/100 ) * ( 24/72 ); // GTH:TODO need to update with proper values. (24/54)
    public static final double k_ArmPositionFactor = k_ArmGearRatio * 2.0 * Math.PI;
    public static final double k_ArmVelocityFactor = k_ArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double k_ArmFreeSpeed = 5676.0 * k_ArmVelocityFactor;
    public static final double k_ArmZeroCosineOffset = 0.873; //GTH:TODO need to get proper angle and convert to raidans, currently at 50degrees

    public static final TrapezoidProfile.Constraints k_ArmMotionConstraint = new TrapezoidProfile.Constraints(1.0, 2.0);
    public static final ArmFeedforward k_ArmFeedForward = new ArmFeedforward( 0.5, 0.25, 3.45, 0.01 );//values taken from team 3467

    /* Arm Motor ID's */
    public static final int k_ArmLeftMotorCanId = 15;
    public static final int k_ArmRightMotorCanId = 16;

    public static final double k_ArmMinOutput = 0;
    public static final double k_ArmMaxOutput = .2; //GTH:TODO need to update value
   
    /* Arm PID constants */
    public static final double k_ArmMotorP = 18;
    public static final double k_ArmMotorI = 0.00; 
    public static final double k_ArmMotorD = 0.02;
    public static final double k_ArmCruise = 4.0;
    public static final double k_ArmAccel  = 10;

    /* Digital Input/Output ID's */
    public static final int k_ArmEncoderId = 0;

    /* Absolute Encoder Offset */
    public static final double k_ArmEncoderOffset = 0.000000; //GTH:TODO need to get value

    /* Pivot Angle Set Points */
    public static final double k_ArmAngleSource = 190; //GTH:TODO need to get value
    public static final double k_ArmAngleAmp    = 190; //GTH:TODO need to get value
    public static final double k_ArmAngleStowed = 270; //GTH:TODO need to get value
    public static final double k_ArmAngleSpeaker = 270; //GTH:TODO need to get value
    public static final double k_ArmAngleStage = 270; //GTH:TODO need to get value
    public static final double k_ArmAnglePass = 270; //GTH:TODO need to get value

  }

  public static class Index
  {
    /* Index Motor ID's */
    public static final int k_IndexMotorCanId = 11;
    
    /* Index Motor Speeds */
    public static final double k_IndexForwardSpeed =  0.6;
    public static final double k_IndexReverseSpeed = -0.2;
  }
}
