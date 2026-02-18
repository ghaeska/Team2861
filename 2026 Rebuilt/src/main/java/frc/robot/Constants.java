// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  public static final class DriveConstants 
  {
    /* PID control values for auto turning */
    public static final double k_turnPID_P = 0.00;
    public static final double k_turnPID_I = 0.00;
    public static final double k_turnPID_D = 0.00;
    public static final double k_turnPID_F = 0.00;

    public static final double k_tolerance_degrees = 2.0f;
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort     = 0;    
    public static final int k2ndDriverControllerPort  = 1;
    public static final double kDriveDeadband         = 0.05; //0.1???
    public static final double k_tolerance_degrees    = 2.0f;
  }


/*************** Subsystem constants go down here. *******************************/

public static final class IntakeConstants
{
  /* Intake Motor ID's */
  public static final int k_LeftIntakeArmMotorCANId   = 11;
  public static final int k_RightIntakeArmMotorCANId  = 12;
  public static final int k_IntakeRollerMotorCANId    = 13;
  
  public static final int k_IntakeArm_MaxCurrent      = 20;
  public static final int k_IntakeRoller_MaxCurrent   = 20;

  /* The following is from a different team not sure how to do it. */
  // public static enum IntakeArmPosition
  // {
  //   STOW( 0 ),          //update to tested value
  //   PARTIAL_STOW( 0 ),  //update to tested value
  //   GROUND( 0 );        //update to tested value
  //   public final int value;
  //   private IntakeArmPosition( int value )
  //   {
  //     this.value = value;
  //   }
  //   public static final double PARTIAL_STOW = 0;
  //   public static final double GROUND = 0;
  //  }

  public static final class IntakeArmSetpoints
  {
    public static final double k_Stow     = 0;  //update to tested value
    public static final double k_MiniStow = 1;  //update to tested value
    public static final double k_Ground   = 2;  //update to tested value
    public static final double k_Jog      = 3;
  }

}

public static final class ShooterConstants
{
  /* Intake Motor ID's */
  public static final int k_LeftShooterMotorCANId   = 10;  //update
  public static final int k_RightShooterMotorCANId  = 14;  //update
  
  public static final int k_Shooter_MaxCurrent      = 40;

  public static final class ShooterSpeedSetpoints
  {
    public static final double k_shoot = 5000;// update
    public static final double k_pass = 5000;//update
    public static final double k_stop = 0;
  }

  public static final Transform3d BALL_TRANSFORM_LEFT = new Transform3d(-0.24, 0.09, 0.5, Rotation3d.kZero);
  public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);
  public static final Transform3d BALL_TRANSFORM_RIGHT = new Transform3d(-0.24, -0.09, 0.5, Rotation3d.kZero);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPS = new InterpolatingDoubleTreeMap();
  static 
  {
    DISTANCE_TO_RPS.put(2.07, 38.0);
    DISTANCE_TO_RPS.put(2.41, 41.0);
    DISTANCE_TO_RPS.put(3.20, 45.0);
    DISTANCE_TO_RPS.put(3.87, 49.0);
    DISTANCE_TO_RPS.put(4.57, 52.0);
    DISTANCE_TO_RPS.put(4.92, 58.0);
  }


  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_SPEED = new InterpolatingDoubleTreeMap();
  static 
  {
    DISTANCE_TO_SHOT_SPEED.put(2.07, 7.0);
    // DISTANCE_TO_SHOT_SPEED.put(2.41, 41.0);
    // DISTANCE_TO_SHOT_SPEED.put(3.20, 45.0);
    // DISTANCE_TO_SHOT_SPEED.put(3.87, 49.0);
    // DISTANCE_TO_SHOT_SPEED.put(4.57, 52.0);
    DISTANCE_TO_SHOT_SPEED.put(4.92, 9.0);
    // DISTANCE_TO_SHOT_SPEED.put(0.0, 7.0);
    // DISTANCE_TO_SHOT_SPEED.put(5.0, 8.25);
    // DISTANCE_TO_SHOT_SPEED.put(10.0, 10.0);
  }

}

public static final class ShooterHoodConstants
{
  /* Intake Motor ID's */
  public static final int k_ShooterHoodMotorCANId   = 30;  //update
  
  public static final int k_ShooterHood_MaxCurrent      = 20;

  public static final class ShooterHoodSetpoints
  {
    public static final double k_HoodMax = 0;// update
    public static final double k_HoodMin = 0;//update
  }

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
  static 
  {
    DISTANCE_TO_HOOD_ANGLE.put(2.07, 2.878);
    DISTANCE_TO_HOOD_ANGLE.put(2.41, 2.853);
    DISTANCE_TO_HOOD_ANGLE.put(3.20, 2.764);
    DISTANCE_TO_HOOD_ANGLE.put(3.87, 2.764);
    DISTANCE_TO_HOOD_ANGLE.put(4.57, 2.764);
    DISTANCE_TO_HOOD_ANGLE.put(4.92, 2.694);
  }

  public static double getHoodAngle(double targetPitch) 
  {
    return targetPitch + Math.PI / 2.0;
  }

  public static final InterpolatingDoubleTreeMap SHOT_ANGLE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
  static 
  {
    SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(59), 2.878);
    SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(45), 2.7);
    SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(40), 2.6);
    SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(38), 2.5);
    SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(37), 2.4);
    SHOT_ANGLE_TO_HOOD_ANGLE.put(Units.degreesToRadians(30), 2.3);
  }
}


public static final class HopperConstants
{
/* Hopper Motor ID */
  public static final int k_HopperMotorCANId   = 20;  //update

  public static final int k_Shooter_MaxCurrent      = 40;
}





}
