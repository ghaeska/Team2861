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

  public static final class ElevatorConstants
  {
    //TODO TODAY: I want you to name all the constants with the following format:

    //  k_Ele_placeholder

    //where k stands for constant
    //where Ele signifies that its for the elevator
    // where placeholder is replaced with what you are defining.

    /* Elevator Motor ID's */
    // TODO TODAY: Define a left elevator motor with a value of 10.  Right motor with value of 11.    

    /* Elevator PID Settings */
    // TODO TODAY: Define PID settings,
    // P has a value of 0.027, I: 0, D: 0, FF: 0.0085
    
    // TODO TODAY: Define MaxVelocity value of 65, Max Acceleration of 200.

    // TODO TODAY: Define Max current of 40.

    // TODO TODAY:  Create values for different heights.  we need the following defined.
    // stow, source, L1, L2, L3, L4, Max, AlgaeScore, Algae High, Algae Low

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
