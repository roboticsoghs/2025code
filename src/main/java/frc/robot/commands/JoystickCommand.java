/*----------------------------------------------------------------------------*/
/* Copyright (c) 2025 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class JoystickCommand extends Command {
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private boolean finished = false;

  /**
   * Creates a new Tankdrive.
   */
  public JoystickCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain, RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(RobotContainer.driveStick.getLeftY()) < 0.05) {
      if (Math.abs(RobotContainer.driveStick.getRightY()) < 0.05) {
        finished = true;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // X, A, B, Y
    if(RobotContainer.driveStick.getXButton()){
      // switches between fast and slow speed
      // Constants.slowMode = !Constants.slowMode;
      RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_3);
      RobotContainer.shooter.shoot_that_fucker();
    }
    if(RobotContainer.driveStick.getYButton()){
      // shoots coral at bottom of reef (L0)
      RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0);
      RobotContainer.shooter.shoot_that_fucker();
    }
    if(RobotContainer.driveStick.getAButton()){
      // shoots coral at L1
      RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_1);
      RobotContainer.shooter.shoot_that_fucker();
    }
    if(RobotContainer.driveStick.getBButton()){
      // shoots coral at L2
      RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_2);
      RobotContainer.shooter.shoot_that_fucker();
    }
    RobotContainer.elevator.setPosition(ElevatorPosition.RESTING_POSITION);

    // Get the value of the left Y axis (left joystick vertical movement)
    double leftAxisValue = -RobotContainer.driveStick.getLeftY();
    if (Math.abs(leftAxisValue) > 0.03) { // Apply a deadband
        leftSpeed = Constants.slowMode
                    ? Constants.slowModeMultipler * leftAxisValue
                    : leftAxisValue;
    } else {
        leftSpeed = 0;
    }

    // Get the value of the right Y axis (right joystick vertical movement)
    double rightAxisValue = -RobotContainer.driveStick.getRightY();
    if (Math.abs(rightAxisValue) > 0.03) { // Apply a deadband
        rightSpeed = Constants.slowMode
                    ? Constants.slowModeMultipler * rightAxisValue
                    : rightAxisValue;
    } else {
        rightSpeed = 0;
    }
    // RobotContainer.drivetrain.arcadeDrive(leftAxisValue, rightAxisValue);

    // Set the motor speeds
    RobotContainer.drivetrain.setLeftSideMotorSpeed(leftSpeed);
    RobotContainer.drivetrain.setRightSideMotorSpeed(rightSpeed);
    // RobotContainer.drivetrain.arcadeDrive(leftSpeed, rightSpeed);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.setLeftSideMotorSpeed(0);
    RobotContainer.drivetrain.setRightSideMotorSpeed(0);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}