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











}
