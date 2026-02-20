package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.Constants.ShooterHood.HoodPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.Superstructure;

public class Controls {
    public static void configureControls(int port, Drivetrain drivetrain, Superstructure superstructure) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -joystick.getTwist()));

        new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05))
                .whileTrue(drivetrain.disableControlledRotateCommand());

        joystick.triggerSoftPress().and(joystick.flipTriggerIn().negate())
                .whileTrue(drivetrain
                        .targetPoseCommand(() -> superstructure.shotCalculator.getCurrentEffectiveTargetPose())
                        .alongWith(superstructure.targetHubSotmCommand()));
        joystick.triggerSoftPress().and(joystick.flipTriggerIn())
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.PI)
                        .alongWith(superstructure.passCommand()));
        joystick.triggerHardPress().whileTrue(
                Commands.parallel(superstructure.hopper.runRollersCommand(),
                        // superstructure.intake.jogUpDownCommand(),
                        superstructure.intake.runRollersCommand()));

        joystick.centerTopHatDown().whileTrue(superstructure.intake.moveToPositionCommand(() -> IntakePosition.GROUND));
        joystick.centerTopHatUp().whileTrue(superstructure.intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.centerBottomHatDown()
                .whileTrue(superstructure.shooterHood.moveToPositionCommand(() -> HoodPosition.BOTTOM));
        joystick.centerBottomHatRight()
                .whileTrue(superstructure.shooterHood.moveToPositionCommand(() -> HoodPosition.MIDDLE));
        joystick.centerBottomHatUp()
                .whileTrue(superstructure.shooterHood.moveToPositionCommand(() -> HoodPosition.TOP));

        joystick.blackThumbButton().whileTrue(Commands.parallel(superstructure.hopper.runRollersCommand()));

        joystick.redButton().whileTrue(Commands.parallel(superstructure.intake.runRollersCommand()));

        joystick.pinkieButton().whileTrue(Commands.parallel(superstructure.intake.reverseRollersCommand(),
                superstructure.hopper.reverseRollersCommand()));

        // Gyro reset field position
        joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

    }

    public static void configureDrivingTestingControls(int port, Drivetrain drivetrain, Superstructure superstructure,
            ShotCalculator shotCalculator) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.povUp().onTrue(Commands.parallel(superstructure.intake.resetPositionCommand(),
                superstructure.shooterHood.resetPositionCommand()).ignoringDisable(true));
        controller.povDown().onTrue(GlobalStates.INITIALIZED.enableCommand().ignoringDisable(true));

        controller.y().whileTrue(superstructure.intake.moveToPositionCommand(() -> IntakePosition.JOG));
        controller.a().whileTrue(superstructure.intake.moveToPositionCommand(() -> IntakePosition.BOTTOM));

        // controller.x().whileTrue(superstructure.shooter.sysIdQuasistatic(Direction.kForward));
        // controller.y().whileTrue(superstructure.shooter.sysIdQuasistatic(Direction.kReverse));
        // controller.a().whileTrue(superstructure.shooter.sysIdDynamic(Direction.kForward));
        // controller.b().whileTrue(superstructure.shooter.sysIdDynamic(Direction.kReverse));

        // controller.x().onTrue(drivetrain.resetGyroCommand());

        // controller.a()
        // .whileTrue(superstructure.shooter.setOverridenSpeedCommand(() ->
        // controller.getRightTriggerAxis()));

        // new Trigger(() -> (Math.abs(controller.getRightX()) > 0.05))
        // .whileTrue(drivetrain.disableControlledRotateCommand());

        // controller.a().onTrue(superstructure.testShotCommand(drivetrain));
        // controller.b().onTrue(superstructure.shootCommand(drivetrain));

        // controller.y().whileTrue(Commands.run(() -> drivetrain.drive(new
        // ChassisSpeeds(0, 0, 100)), drivetrain));

        // controller.leftBumper()
        // .whileTrue(drivetrain.targetPoseCommand(shotCalculator::getCurrentEffectiveTargetPose));
        // controller.leftBumper()
        // .whileTrue(drivetrain.controlledRotateCommand(shotCalculator::getCurrentEffectiveYaw));

        // controller.povUp().whileTrue(drivetrain.controlledRotateCommand(() -> 0));
        // controller.povDown().whileTrue(drivetrain.controlledRotateCommand(() ->
        // Math.PI));
        // controller.povRight().whileTrue(drivetrain.controlledRotateCommand(() -> 3 *
        // Math.PI / 2.0));
        // controller.povLeft().whileTrue(drivetrain.controlledRotateCommand(() ->
        // Math.PI
        // / 2.0));

        // superstructure.intake
        // .setRollerVoltageCommand(
        // () -> (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) *
        // 12)

        // controller.x().whileTrue(superstructure.intake.runRollersCommand());
        // controller.x().whileTrue(superstructure.intake.reverseRollersCommand());

    }
}
