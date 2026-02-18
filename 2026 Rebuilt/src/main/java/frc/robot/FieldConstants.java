package frc.robot;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;


import frc.robot.utils.BallConstants;
import frc.robot.utils.Utils;
import frc.robot.utils.Reflector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.22);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317.69);

    public static Pose2d rotateBluePoseIfNecessary(Pose2d original) 
    {
      return Utils.shouldFlipValueToRed()
              ? Reflector.rotatePoseAcrossField(original, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH)
              : original;
    }

    public static final BallConstants BALL_CONSTANTS = new BallConstants(
            Grams.of(210).in(Kilograms), Inches.of(3).in(Meters), 1.2, 0.30, 1.2, 0.35, 9.81, 20);

    public static final class Hub 
    {
      public static final Pose3d CENTER = new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);
    }
}