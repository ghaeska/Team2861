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
  public static final int k_LeftIntakeArmMotorCANId   = 10;
  public static final int k_RightIntakeArmMotorCANId  = 11;
  public static final int k_IntakeRollerMotorCANId    = 12;
  
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
  }

}

public static final class ShooterConstants
{
  /* Intake Motor ID's */
  public static final int k_LeftShooterMotorCANId   = 10;  //update
  public static final int k_RightShooterMotorCANId  = 11;  //update
  
  public static final int k_Shooter_MaxCurrent      = 40;

  public static final class ShooterSpeedSetpoints
  {
    public static final double k_shoot = 5000;// update
    public static final double k_pass = 5000;//update
  }
}







}
