// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

    /* Intake Beam Break Digital Input ID */
    public static final int k_DIO_IntakeSensorID = 3;

    /* Intake Speeds */
    /* positive value ejects, negative intakes. */
    public static final double k_IntakeIntakeSpeedSlow = 0.6; 
    public static final double k_IntakeIntakeSpeedFast = 1.0; 
    public static final double k_IntakeEjectSpeed = -0.3;
  }

  public static class Shooter
  {
    /* Shooter PID constants */
    public static final double k_ShooterMotorP = 0.00025;
    public static final double k_ShooterMotorI = 0.0;
    public static final double k_ShooterMotorD = 0.00001;
    public static final double k_ShooterMotorFF = 0.000159;

    /* Shooter Motor ID's */
    public static final int k_ShooterTopMotorCanId = 12;
    public static final int k_ShooterBotMotorCanId = 13;

    /* Shooter Min/Max Outputs */
    public static final double k_ShooterMinOutput = 0;
    public static final double k_ShooterMaxOutput = 1;
    
    /* Shooter Speeds (RPM) */
    public static final double k_ShooterSpeed_Speaker = 2250; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Amp = 0;
    //public static final double k_ShooterSpeed_Stage = 4000;
    public static final double k_ShooterSpeed_Pass = 1000; //GTH:TODO need to update value
  }

  public static class Arm
  {
    /* Arm Motor ID's */
    public static final int k_ArmLeftMotorCanId = 15;
    public static final int k_ArmRightMotorCanId = 16;

    public static final double k_ArmMinOutput = -0.5; //GTH:TODO need to update value
    public static final double k_ArmMaxOutput =  0.5; //GTH:TODO need to update value
   
    /* Arm PID constants */
    public static final double k_ArmMotorP  = .05;
    public static final double k_ArmMotorI  = 0.00; 
    public static final double k_ArmMotorD  = 0.00;
    public static final double k_ArmMotorFF = 0.001;

    /* Pivot Angle Set Points */
    public static final Rotation2d k_ArmAngleAmp    = Rotation2d.fromDegrees(330); //GTH:TODO need to update values every time chain skips
    public static final Rotation2d k_ArmAngleStowed = Rotation2d.fromDegrees(264); //GTH:TODO need to update values every time chain skips
    public static final Rotation2d k_ArmAngleHang = Rotation2d.fromDegrees(340);  //GTH:TODO need to get value
  }

  public static class Index
  {
    /* Index Motor ID's */
    public static final int k_IndexMotorCanId = 11;

    /* Index Beam Break Digital Input ID */
    public static final int k_DIO_IndexSensorID = 4;
    
    /* Index Motor Speeds */
    public static final double k_IndexForwardSpeed =  0.4;
    public static final double k_IndexReverseSpeed = -0.4;
  }
}
