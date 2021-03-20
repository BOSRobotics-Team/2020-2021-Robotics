package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

/** Drives a set distance using a motion profile. */
public class DriveMotionProfileArcCommand extends CommandBase {
  public final DriveTrain m_driveTrain;

  /**
   * Creates a new DriveDistanceProfiledCommand.
   *
   * @param meters The distance to drive.
   * @param drive The drive subsystem to use.
   */
  public DriveMotionProfileArcCommand(double meters, double angle, DriveTrain drive) {
    m_driveTrain = drive;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_driveTrain.resetPosition();
    m_driveTrain.zeroHeading();

    m_driveTrain.setDriveMode(DriveMode.ARCADE);
    m_driveTrain.setUseSquares(true);
    m_driveTrain.setDriveScaling(1.0);
    m_driveTrain.enableBrakes(true);
    m_driveTrain.enableDriveTrain(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    //m_driveTrain.drive(m_controller);
    //m_driveTrain.logPeriodic();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.tankDriveVolts(0, 0);
    m_driveTrain.setUseSquares(true);
    m_driveTrain.enableBrakes(true);
    m_driveTrain.setDriveScaling(1.0);
    m_driveTrain.setDriveMode(DriveMode.ARCADE);
    m_driveTrain.enableDriveTrain(false);
}

 // Make this return true when this Command no longer needs to run execute()
@Override
public boolean isFinished() {
    return false;
 }
}