package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utils 
{
  public static boolean shouldFlipValueToRed() 
  {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
