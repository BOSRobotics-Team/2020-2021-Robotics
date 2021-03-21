// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import oi.limelightvision.limelight.frc.LimeLight;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> chooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain = new DriveTrain();
  public final Shooter shooter = new Shooter();
  public final Hopper hopper = new Hopper();
  public final Intake intake = new Intake();
  public final Hook hook = new Hook();
  public final Winch winch = new Winch();

  public final UnjamIntakeCommand m_unJamIntakeCommand = new UnjamIntakeCommand(intake);
  public final IntakeCommand m_intakeCommand = new IntakeCommand(intake);
  public final ExtendHookCommand m_extendHookCommand = new ExtendHookCommand(hook);
  public final RetrackHookCommand m_retractHookCommand = new RetrackHookCommand(hook);
  public final ReverseWinchCommand m_reverseWinchCommand = new ReverseWinchCommand(winch);
  public final RetrackWinchCommand m_retractWinchCommand = new RetrackWinchCommand(winch);
  public final HopperCommand m_hopperCommand = new HopperCommand(hopper);
  public final UnjamHopperCommand m_unjamHopperCommand = new UnjamHopperCommand(hopper);
  public final ShootCommand m_shootCommand = new ShootCommand(shooter);
  public final SpinUpShooterCommand m_spinupShooterCommand = new SpinUpShooterCommand(shooter);
  public final RunHopperAutoCommand m_runHopperAutoCommand = new RunHopperAutoCommand(hopper);

  //Driver Controller
  public final XboxController driverController = new XboxController(0);
  public final JoystickButton a_Button_Driver = new JoystickButton(driverController, 1);
  public final JoystickButton b_Button_Driver = new JoystickButton(driverController, 2);
  public final JoystickButton x_Button_Driver = new JoystickButton(driverController, 3);
  public final JoystickButton y_Button_Driver = new JoystickButton(driverController, 4);
  public final JoystickButton left_Bumper_Driver = new JoystickButton(driverController, 5);
  public final JoystickButton right_Bumper_Driver = new JoystickButton(driverController, 6);
  public final JoystickButton back_Button_Driver = new JoystickButton(driverController, 7);
  public final JoystickButton start_Button_Driver = new JoystickButton(driverController, 8);
  public final JoystickButton left_Stick_Driver = new JoystickButton(driverController, 9);
  public final JoystickButton right_Stick_Driver = new JoystickButton(driverController, 10);
  
  //Operator Controller
  public final XboxController operatorController = new XboxController(1);
  public final JoystickButton a_Button_Operator = new JoystickButton(operatorController, 1);
  public final JoystickButton b_Button_Operator = new JoystickButton(operatorController, 2);
  public final JoystickButton x_Button_Operator = new JoystickButton(operatorController, 3);
  public final JoystickButton y_Button_Operator = new JoystickButton(operatorController, 4);
  public final JoystickButton left_Bumper_Operator = new JoystickButton(operatorController, 5);
  public final JoystickButton right_Bumper_Operator = new JoystickButton(operatorController, 6);
  public final JoystickButton back_Button_Operator = new JoystickButton(operatorController, 7);
  public final JoystickButton start_Button_Operator = new JoystickButton(operatorController, 8);
  public final JoystickButton left_Stick_Operator = new JoystickButton(operatorController, 9);
  public final JoystickButton right_Stick_Operator = new JoystickButton(operatorController, 10);
  
  //Camera
  public final LimeLight limeLight = new LimeLight();
  public final UsbCamera cam0; 
  public final UsbCamera cam1; 

  public final AutonomousCommand m_autoCommand = new AutonomousCommand(this);
  public final CommandDriveTrain m_cmdDriveTrainCommand = new CommandDriveTrain(driveTrain, driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(m_cmdDriveTrainCommand);
    
    // Add commands to Autonomous Sendable Chooser
    chooser.setDefaultOption("Autonomous Command", m_autoCommand);
    chooser.addOption("AutoDriveStraight Command", new AutoDriveStraightCommand(this));
    chooser.addOption("AutoDriveTurn Command", new AutoDriveTurnCommand(this));

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", m_autoCommand);
    SmartDashboard.putData("CommandDriveTrain", m_cmdDriveTrainCommand);
    SmartDashboard.putData("Auto mode", chooser);

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

    cam0 = CameraServer.getInstance().startAutomaticCapture(0);
    cam1 = CameraServer.getInstance().startAutomaticCapture(1);  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    a_Button_Driver.whileHeld(m_unJamIntakeCommand);    
    left_Bumper_Driver.whileHeld(m_intakeCommand);
    right_Bumper_Driver.whenPressed(() -> driveTrain.setMaxOutput(0.5))
                       .whenReleased(() -> driveTrain.setMaxOutput(1.0));
    back_Button_Driver.whenPressed(() -> driveTrain.toggleDriveMode());
    right_Stick_Driver.whenPressed(() -> driveTrain.setUseSquares(!driveTrain.getUseSquares()));

    b_Button_Driver.whenPressed(new DriveDistanceProfiledCommand(3, driveTrain).withTimeout(10));

    a_Button_Operator.whileHeld(m_extendHookCommand);
    b_Button_Operator.whileHeld(m_retractHookCommand);
    y_Button_Operator.whileHeld(m_reverseWinchCommand);
    x_Button_Operator.whileHeld(m_retractWinchCommand);
    left_Bumper_Operator.whileHeld(m_hopperCommand);
    right_Bumper_Operator.whileHeld(m_shootCommand);
    back_Button_Operator.whileHeld(m_unjamHopperCommand);    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public XboxController getDriverController() {
    return driverController;
  }
}
