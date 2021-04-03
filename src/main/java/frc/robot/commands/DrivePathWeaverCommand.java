package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveMode;

/** Drives a set distance using a motion profile. */
public class DrivePathWeaverCommand extends CommandBase {
  public final DriveTrain m_driveTrain;
  public final String m_pathFilename;
  public RamseteCommand mRamseteCommand;

  private Trajectory m_trajectory = new Trajectory();

  /**
   * Creates a new DriveDistanceProfiledCommand.
   *
   * @param meters The distance to drive.
   * @param drive The drive subsystem to use.
   */
  public DrivePathWeaverCommand(String pathFilename, DriveTrain drive) {
    m_driveTrain = drive;
    m_pathFilename = pathFilename; // "paths/YourPath.wpilib.json";
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    TrajectoryConstraint constraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
          m_driveTrain.getDriveKinematics(),
        10);

    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
      Constants.kMaxSpeedMetersPerSecond,
      Constants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(m_driveTrain.getDriveKinematics())
    // Apply the voltage constraint
    .addConstraint(constraint);

    this.readPathWeaverFile(m_pathFilename);

    m_trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    m_driveTrain.resetPosition();
    m_driveTrain.zeroHeading();

    m_driveTrain.setDriveMode(DriveMode.ARCADE);
    m_driveTrain.setUseSquares(true);
    m_driveTrain.setDriveScaling(1.0);
    m_driveTrain.enableBrakes(true);
    m_driveTrain.enableDriveTrain(true);

    mRamseteCommand =
    new RamseteCommand(
        m_trajectory,
        m_driveTrain::getCurrentPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
            m_driveTrain.getDriveKinematics(),
            m_driveTrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveTrain::tankDriveVolts,
        m_driveTrain);

        m_driveTrain.resetOdometry(m_trajectory.getInitialPose());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    mRamseteCommand.execute();
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

  public void readPathWeaverFile(String trajectoryJSON)
  {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }
}