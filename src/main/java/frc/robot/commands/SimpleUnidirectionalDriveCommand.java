package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

/** Very simple unidirectional drive control. */
public class SimpleUnidirectionalDriveCommand extends CommandBase {

  /** The OI used for input. */
  public final XboxController controller;

  /** The subsystem to execute this command on. */
  private final DriveTrain subsystem;

  /**
   * Default constructor
   *
   * @param subsystem The subsystem to execute this command on
   * @param controller The OI that gives the input to this command.
   */
  public SimpleUnidirectionalDriveCommand(DriveTrain subsystem, XboxController controller) {
    this.controller = controller;
    this.subsystem = subsystem;
    // Default commands need to require their subsystems.
    addRequirements(subsystem);
  }

  /** Stop the drive for safety reasons. */
  @Override
  public void initialize() {
    subsystem.fullStop();
  }

  /** Give output to the motors based on the stick inputs. */
  @Override
  public void execute() {
    subsystem.setOutput(this.controller);
  }

  /**
   * Run constantly because this is a default drive
   *
   * @return false
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Log and brake when interrupted. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "SimpleUnidirectionalDrive Interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    // Brake for safety!
    subsystem.fullStop();
  }
}
