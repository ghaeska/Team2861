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
  public static final class DriveConstants 
  {
    
  }

  public static class Intake
  {
    /* Motor ID's */
    public static final int k_IntakeMotorCanId = 10;
    public static final int k_PivotMotorCanId = 11;

    /* Digital Input/Output ID's */
    public static final int k_PivotEncoderId = 0;
    public static final int k_IntakeLimitSwitchId = 2;

    /* Absolute Encoder Offset */
    public static final double k_PivotEncoderOffset = 0.000000;  //GTH:TODO, NEED TO GET ENCODER OFFSET WHEN INTAKE IS STRAIGHT UP IN THE AIR.

    /* Pivot Angle Set Points */
    public static final double k_PivotAngleGround = 60;
    public static final double k_PivotAngleSource = 190;
    public static final double k_PivotAngleAmp    = 190;
    public static final double k_PivotAngleStowed = 270;

    /* Intake Speeds */
    public static final double k_IntakeIntakeSpeed = 0.7;
    public static final double k_IntakeEjectSpeed = -0.45;
    public static final double k_IntakeFeedShootSpeed = -0.50;
  }  
}
