// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // initialize subsystems
  public static final DriveTrain drivetrain = new DriveTrain();
  public static final Elevator elevator = new Elevator();
  public static final Shooter shooter = new Shooter();
  public static final XboxController driveStick = new XboxController(0);
  public static final Joystick m_operator = new Joystick(1);
  public static final Vision visionSystem = new Vision();

  // operator board buttons
  public static JoystickButton shooterButton = new JoystickButton(m_operator, Constants.SLOWMODE_PORT);
  public static JoystickButton leftAlignButton = new JoystickButton(m_operator, Constants.LEFT_ALIGN_PORT);
  public static JoystickButton rightAlignButton = new JoystickButton(m_operator, Constants.RIGHT_ALIGN_PORT);
  public static JoystickButton centerAlignButton = new JoystickButton(m_operator, Constants.CENTER_ALIGN_PORT);
  public static JoystickButton unstuckCoralButton = new JoystickButton(m_operator, Constants.UNSTUCK_CORAL_PORT);
  // public static JoystickButton abortButton = new JoystickButton(m_operator, Constants.ABORT_PORT);
  // public static JoystickButton intakeStart = new JoystickButton(driveStick, 0)

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutos();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  private void configureAutos() {
    autoChooser.setDefaultOption("CENTER LEAVE", AutoCommand.Auto1_CENTER());
    autoChooser.addOption("1P LEFT", AutoCommand.Auto3_LEFT());
    autoChooser.addOption("1P RIGHT", AutoCommand.Auto2_RIGHT());

    SmartDashboard.putData("Auto selector", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
    // return AutoCommand.Auto2_RIGHT();
  }

  // public Command testAuto() {
  //   return Commands.sequence(
  //     Commands.runOnce(() -> drivetrain.setAllMotorsSpeed(0.5, 0.5)),
  //     Commands.waitSeconds(0.5),
  //     Commands.runOnce(() -> drivetrain.setAllMotorsSpeed(0, 0))
  //   );
  // }
}