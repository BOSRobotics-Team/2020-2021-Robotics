package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.*;

public class EnableMotorsCommand extends InstantCommand {
    /** The subsystem to execute this command on. */
    private final DriveTrain subsystem;
    
    /**
     * Default constructor
     *
     * @param subsystem The subsystem to execute this command on.
     */
    public EnableMotorsCommand(DriveTrain subsystem) {
        this.subsystem = subsystem;
    }
    
    /** Log when this command is initialized */
    @Override
    public void initialize() {
        Shuffleboard.addEventMarker("EnableMotors init.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
    
    /** Do the state change. */
    @Override
    public void execute() {
        subsystem.enableDriveTrain(true);
    }
    
    /** Log when this command ends */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Shuffleboard.addEventMarker("EnableMotors Interrupted!", this.getClass().getSimpleName(), EventImportance.kNormal);
        }
        Shuffleboard.addEventMarker("EnableMotors end.", this.getClass().getSimpleName(), EventImportance.kNormal);
    }
}
