// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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
    public static final double k_IntakeIntakeSpeedSlow = -0.3; //GTH:TODO need to update value
    public static final double k_IntakeIntakeSpeedFast = -0.8; //GTH:TODO need to update value
    public static final double k_IntakeEjectSpeed = 0.2; //GTH:TODO need to update value
    public static final double k_IntakeFeedShootSpeed = -0.50; //GTH:TODO need to update value
  }

  public static class Shooter
  {
    /* Shooter PID constants */
    public static final double k_ShooterMotorP = 0.00005;
    public static final double k_ShooterMotorI = 0.0;
    public static final double k_ShooterMotorD = 0.0;
    public static final double k_ShooterMotorFF = 0.002;

    /* Shooter Motor ID's */
    public static final int k_ShooterTopMotorCanId = 12;
    public static final int k_ShooterBotMotorCanId = 13;

    /* Shooter Arm Motor ID's */
    public static final int k_ShooterArmLeftMotorCanId = 15;
    public static final int k_ShooterArmRightMotorCanId = 16;

    /* Shooter Min/Max Outputs */
    public static final double k_ShooterMinOutput = 0;
    public static final double k_ShooterMaxOutput = 1;

    public static final double k_ShooterArmMinOutput = 0;
    public static final double k_ShooterArmMaxOutput = .2; //GTH:TODO need to update value

    /* Shooter Speeds */
    public static final double k_ShooterSpeed_Speaker = 0.1; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Amp = 0.1; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Stage = 0.1; //GTH:TODO need to update value
    public static final double k_ShooterSpeed_Pass = 0.1; //GTH:TODO need to update value

    /* Pivot Angle Set Points */
    public static final double k_ShooterArmAngleSource = 190; //GTH:TODO need to get value
    public static final double k_ShooterArmAngleAmp    = 190; //GTH:TODO need to get value
    public static final double k_ShooterArmAngleStowed = 270; //GTH:TODO need to get value
    public static final double k_ShooterArmAngleSpeaker = 270; //GTH:TODO need to get value
    public static final double k_ShooterArmAngleStage = 270; //GTH:TODO need to get value
    public static final double k_ShooterArmAnglePass = 270; //GTH:TODO need to get value


    /* Digital Input/Output ID's */
    public static final int k_ShooterArmEncoderId = 0;

    /* Absolute Encoder Offset */
    public static final double k_ShooterArmEncoderOffset = 0.000000; //GTH:TODO need to get value

    /* Intake PID constants */
    public static final double k_ShooterArmMotorP = 0.12;
    public static final double k_ShooterArmMotorI = 0.0;
    public static final double k_ShooterArmMotorD = 0.001;
  }
}
