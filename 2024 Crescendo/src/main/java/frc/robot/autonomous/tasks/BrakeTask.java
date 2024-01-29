package frc.robot.autonomous.tasks;

import frc.robot.subsystems.DriveTrain;

public class BrakeTask extends Task {
  private DriveTrain m_drive;

  public BrakeTask() {
    m_drive = DriveTrain.getInstance();
  }

  @Override
  public void start() {
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    m_drive.drive(0, 0, 0, true, false);
  }
}
