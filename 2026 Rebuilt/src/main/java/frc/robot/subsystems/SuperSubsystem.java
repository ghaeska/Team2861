package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.utils.BallState;

import static frc.robot.Constants.ShooterConstants.BALL_TRANSFORM_LEFT;
import static frc.robot.Constants.ShooterConstants.BALL_TRANSFORM_RIGHT;
import static frc.robot.Constants.ShooterConstants.DISTANCE_TO_RPS;
import static frc.robot.Constants.ShooterHoodConstants.DISTANCE_TO_HOOD_ANGLE;
import static frc.robot.Constants.ShooterHoodConstants.SHOT_ANGLE_TO_HOOD_ANGLE;

public class SuperSubsystem extends SubsystemBase
{
  public final ShooterSubsystem shooter;
  public final ShooterHoodSubsystem shooterHood;
  public final HopperSubsystem hopper;
  public final IntakeSubsystem intake;

  public ShotCalcSubsystem shotCalc;

  private double testShotVelocity = 0;

  private double testLaunchPitch = 0;

    private Supplier<Double> distanceSupplier;
    private Supplier<Double> distanceSotmSupplier;

  public SuperSubsystem( DriveSubsystem driveSubsystem, 
                         ShooterSubsystem shooter, 
                         ShooterHoodSubsystem shooterHood, 
                         HopperSubsystem hopper, 
                         IntakeSubsystem intake,
                         ShotCalcSubsystem shotCalc ) 
  {
    this.shooter = shooter;
    this.shooterHood = shooterHood;
    this.hopper = hopper;
    this.intake = intake;
    this.shotCalc = shotCalc;
    distanceSupplier = () -> driveSubsystem.getPose().getTranslation()
            .getDistance(FieldConstants.Hub.CENTER.getTranslation().toTranslation2d());
    distanceSotmSupplier = () -> driveSubsystem.getPose().getTranslation()
            .getDistance(shotCalc.getCurrentEffectiveTargetPose().getTranslation().toTranslation2d());
  }

  public void periodic() 
  {

  }

  public Command useRequirement() 
  {
    return runOnce(() -> 
    {
      
    });
  }

  public Command targetHubEasyCommand() 
  {
    return Commands.parallel(
            shooter.spinAtVelocityCommand(
                    () -> DISTANCE_TO_RPS.get(distanceSupplier.get())).asProxy(),
            shooterHood.moveToSpecificPositionCmd(
                    () -> DISTANCE_TO_HOOD_ANGLE.get(distanceSupplier.get())).asProxy())
            .andThen(useRequirement());
  }

  public Command targetHubSotmCommand() 
  {
    return Commands.parallel(
            shooter.spinAtVelocityCommand(
                    () -> DISTANCE_TO_RPS.get(distanceSotmSupplier.get())).asProxy(),
            shooterHood.moveToSpecificPositionCmd(
                    () -> DISTANCE_TO_HOOD_ANGLE.get(distanceSotmSupplier.get())).asProxy())
            .andThen(useRequirement());
  }

  public Command passCommand() 
  {
    return Commands.parallel(
            shooter.spinAtVelocityCommand(() -> 70.0).asProxy(),
            shooterHood.moveToSpecificPositionCmd(
                    () -> SHOT_ANGLE_TO_HOOD_ANGLE.get(Units.degreesToRadians(40.0))).asProxy())
            .andThen(useRequirement());
  }

  public Command testShotCommand(DriveSubsystem drivetrain) 
  {
    return runOnce(() -> {
        Pose2d drivetrainPose = drivetrain.getPose();
        ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldRelativeSpeeds();

        shootBall(drivetrainPose, drivetrainSpeeds, testShotVelocity, Units.degreesToRadians(testLaunchPitch));
    });
  }

  public void shootBall(Pose2d drivetrainPose, ChassisSpeeds fieldRelChassisSpeeds, double shotVelocity,
          double launchPitch) 
  {
    Translation3d drivetrainVeloTransform = new Translation3d(fieldRelChassisSpeeds.vxMetersPerSecond,
            fieldRelChassisSpeeds.vyMetersPerSecond, 0);

    // ballSimulator
    //         .addBall(new BallState(
    //                 new Pose3d(new Translation3d(drivetrainPose.getX(), drivetrainPose.getY(), 0),
    //                         new Rotation3d(drivetrainPose.getRotation())).plus(BALL_TRANSFORM_LEFT),
    //                 new Translation3d(
    //                         shotVelocity * Math.cos(launchPitch),
    //                         0,
    //                         shotVelocity * Math.sin(launchPitch))
    //                         .rotateBy(new Rotation3d(drivetrainPose.getRotation()))
    //                         .plus(drivetrainVeloTransform),
    //                 new Translation3d(0, 100, 0)));
    // ballSimulator
    //         .addBall(new BallState(
    //                 new Pose3d(new Translation3d(drivetrainPose.getX(), drivetrainPose.getY(), 0),
    //                         new Rotation3d(drivetrainPose.getRotation())).plus(BALL_TRANSFORM_RIGHT),
    //                 new Translation3d(
    //                         shotVelocity * Math.cos(launchPitch),
    //                         0,
    //                         shotVelocity * Math.sin(launchPitch))
    //                         .rotateBy(new Rotation3d(drivetrainPose.getRotation()))
    //                         .plus(drivetrainVeloTransform),
    //                 new Translation3d(0, 100, 0)));
  }






}
