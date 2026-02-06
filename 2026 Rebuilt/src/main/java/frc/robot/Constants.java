// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.util.Units;

/*
**  The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
**  constants. This class should not be used for any other purpose. All constants should be declared
**  globally (i.e. public static). Do not put anything functional in this class.
** 
**  <p>It is advised to statically import this class (or one of its inner classes) wherever the
**  constants are needed, to reduce verbosity.
*/
public final class Constants 
{
  public static class Field 
  {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double k_tolerance_degrees = 2.0f;
  }

/* Subsystem constants go down here. */

  public static final class ShooterConstants
  {
    /* Shooter Motor ID's */
    public static final int k_LeftShooterMotorCANId = 14; //Redefine
    public static final int k_RightShooterMotorCANId = 10; //Redefine

    public static final int k_Shooter_MaxCurrent = 40;

    /* Pivot Angle Set points */
    public static final class ShooterSetpoints
    {
      public static final double k_pass = 5000;
      public static final double k_shoot = 5000;
  
    }

  }

public static final class FeedConstants
  {
    /* Feed Motor ID's */
    
    public static final int k_FeedIntMotorCANId = 17; //Redefine

    public static final int k_Feed_MaxCurrent = 40;

    /* Because the Coral is on a pivot, we need to have a PID to hold the position */
    public static final double k_FeedIntMotorP  = 0.006;
    public static final double k_FeedIntMotorI  = 0.00; 
    public static final double k_FeedIntMotorD  = 0.00;
    public static final double k_FeedIntMotorFF = 0.00;

    public static final double k_PivotMinOutput = -0.5;
    public static final double k_PivotMaxOutput = 0.5;

    /* Absolute Encoder for knowing exact angle */
    public static final int k_FeedIntEncoderId = 0;

    /* Absolute Encoder Offset */
    public static final double k_FeedIntEncoderOffset = 0.000000;

    /* Pivot Angle Set points */
    public static final class FeedIntSetpoints
    {
       public static final double k_Stow = 170;/* Redefine */
    }
  }

public static final class IntakeConstants
  {
    /* Intake Motor ID's */
    public static final int k_LeftIntakeMotorCANId = 10; //Redefine
    public static final int k_RightIntakeMotorCANId = 11; //Redefine

    /* Intake PID Settings */
    public static final double k_Int_PID_P = 0.012;
    public static final double k_Int_PID_I = 0;
    public static final double k_Int_PID_D = 0.0;
    public static final double k_Int_PID_FF = 0.0;

    public static final double k_Int_MaxVelocity = 65;
    public static final double k_Int_MaxAcceleration = 200;

    public static final int k_Int_MaxCurrent = 40;

    /* Pivot Angle Set points */
    public static final class IntakeSetpoints
    {
       public static final double k_Stow = 170;/* Redefine */
       public static final double k_MiniStow = 170;/* Redefine */
       public static final double k_Ground = 170;/* Redefine */
    }
  }









}
