package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.*;
import frc.robot.subsystems.*;

/** Drives a set distance using a motion profile. */
public class DriveDistanceProfiledCommand extends TrapezoidProfileCommand {
  /**
   * Creates a new DriveDistanceProfiledCommand.
   *
   * @param meters The distance to drive.
   * @param drive The drive subsystem to use.
   */
  public DriveDistanceProfiledCommand(double meters, DriveTrain drive) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared),
            // End at desired position in meters; implicitly starts at 0
            new TrapezoidProfile.State(meters, 0)),
        // Pipe the profile state to the drive
        setpointState -> drive.driveToTarget(setpointState.position),
        // Require the drive
        drive);
    // Reset drive encoders since we're starting at 0
    drive.resetPosition();
  }
}