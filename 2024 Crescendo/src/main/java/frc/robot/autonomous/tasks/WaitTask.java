package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;

public class WaitTask extends Task {
  private Timer m_runningTimer = new Timer();
  private double m_targetTime;

  private DriveTrain m_swerve = DriveTrain.getInstance();

  public WaitTask(double timeSeconds) {
    m_targetTime = timeSeconds;
  }

  @Override
  public void start() {
    m_runningTimer.start();
  }

  @Override
  public void update() {
    m_swerve.drive( 0, 0, 0, true, false );
  }

  @Override
  public boolean isFinished() {
    return m_runningTimer.get() >= m_targetTime;
  }
}
