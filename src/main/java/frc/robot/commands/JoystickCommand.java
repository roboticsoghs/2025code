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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickCommand extends Command {
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private boolean slowModeMultiplier = false;
  private boolean finished = false;
  private boolean ignoreOp;
  private boolean ignoreDrive;
  private double throttleValue;
  private double centerMultiplier;
  // private final POVButton dpadRight = new POVButton(driveStick, 90);
  // private final POVButton dpadDown = new POVButton(RobotContainer.driveStick, 180);
  // private final POVButton dpadLeft = new POVButton(driveStick, 270);

  /**
   * Creates a new Tankdrive.
   */
  public JoystickCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.drivetrain, RobotContainer.elevator);
    addRequirements(RobotContainer.drivetrain, RobotContainer.shooter, RobotContainer.elevator);
    // addRequirements(RobotContainer.drivetrain);
    throttleValue = 0;
    ignoreOp = false;
    slowModeMultiplier = false;
    ignoreDrive = false;
    centerMultiplier = 1;
    // addRequirements(RobotContainer.elevator);
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
    if(RobotContainer.driveStick.getAButtonPressed()) {
      slowModeMultiplier = !slowModeMultiplier;
      SmartDashboard.putBoolean("slowMode", slowModeMultiplier);
    } else {
      SmartDashboard.putBoolean("slow mode: ", slowModeMultiplier);
    }
    if(RobotContainer.centerAlignButton.getAsBoolean()) {
      ignoreDrive = true;
      ignoreOp = true;
      RobotContainer.visionSystem.alignToReef(Constants.reefCenter);
      try {
        Thread.sleep(750);
      } catch(InterruptedException e) {
          e.printStackTrace();
      }
      centerMultiplier = 0.4;
      // triggerPressed(0.4);
      // ignoreDrive = false;
      // ignoreOp = false;
    } else
    if(RobotContainer.leftAlignButton.getAsBoolean()) {
      ignoreDrive = true;
      ignoreOp = true;
      RobotContainer.visionSystem.alignToReef(Constants.leftAlignReef);
      try {
        Thread.sleep(750);
      } catch(InterruptedException e) {
          e.printStackTrace();
      }
      // triggerPressed(0.4);
      // ignoreDrive = false;
      // ignoreOp = false;
    } else 
    if(RobotContainer.rightAlignButton.getAsBoolean()) {
      ignoreDrive = true;
      ignoreOp = true;
      RobotContainer.visionSystem.alignToReef(Constants.rightAlignReef);
      try {
        Thread.sleep(750);
      } catch(InterruptedException e) {
          e.printStackTrace();
      }
      // triggerPressed(0.5);
      // ignoreDrive = false;
      // ignoreOp = false;
    }

    if(RobotContainer.m_operator.isConnected()){
      // switches between fast and slow speed
      throttleValue = RobotContainer.m_operator.getRawAxis(6);
      SmartDashboard.putNumber("op board 6: ", throttleValue);
      // Constants.slowMode = !Constants.slowMode;
      // RobotContainer.elevator.setPosition(ElevatorPosition.RESTING_POSITION);
      // RobotContainer.shooter.shoot_that_fucker(0); // TODO: get value
    }
    if (!ignoreOp) {
      if(throttleValue <= -0.9){
        // rest
        // shoots coral at bottom of reef (L0)
        RobotContainer.elevator.setPosition(ElevatorPosition.RESTING_POSITION);
        // RobotContainer.shooter.shoot_that_fucker(0);// TODO: get value
      }
      if(throttleValue <= -0.7 && throttleValue >= -0.8){
        // L0
        // shoots coral at L0
        RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0);
      }
      if(throttleValue <= -0.5 && throttleValue >= -0.6){
        // L1
        // shoots coral at L1
        RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_1);
      }
      if(throttleValue <= -0.3 && throttleValue >= -0.4){
        // shoots coral at L2
        RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_2);
      }
      if(throttleValue <= -0.1 && throttleValue >= -0.2){
        RobotContainer.elevator.setPosition(ElevatorPosition.INTAKE_POSITION);
      }
    }
        // Check if the right trigger is pressed beyond a threshold (e.g., 0.5)
    
    if (RobotContainer.driveStick.getLeftTriggerAxis() < 0.5) {
      SmartDashboard.putBoolean("Left click", false);
      triggerPressed(0);
    }
    if (RobotContainer.driveStick.getRightTriggerAxis() < 0.5) {
      SmartDashboard.putBoolean("Right click", false);
      triggerPressed(0);
    }
    // if (!RobotContainer.shooterButton.getAsBoolean()) {
    //   SmartDashboard.putBoolean("Right click", false);
    //   triggerPressed(0);
    // }
    SmartDashboard.putNumber("click", RobotContainer.driveStick.getRightTriggerAxis());
    // // Check if the right trigger is pressed beyond a threshold (e.g., 0.5)
    if (RobotContainer.driveStick.getLeftTriggerAxis() > 0.5) {
      SmartDashboard.putBoolean("Left click", true);
      triggerPressed(-0.15);
    }
    if (RobotContainer.driveStick.getRightTriggerAxis() > 0.5) {
      SmartDashboard.putBoolean("Right click", true);
      handleRightTriggerPressed(0.4 * centerMultiplier); // 25% speed
      ignoreOp = false;
      ignoreDrive = false;
    }
    // if (RobotContainer.shooterButton.getAsBoolean()) {
    //   SmartDashboard.putBoolean("Right click", true);
    //   handleRightTriggerPressed(0.5);
    // }


    if (RobotContainer.driveStick.getLeftBumperButton()) {
      intakeStart();
    }
    if (RobotContainer.driveStick.getRightBumperButton()) {
      intakeStop();
    }

    switch(RobotContainer.driveStick.getPOV()) {
      case 0:
        ignoreOp = true;
        handleUp();
        break;
      case 180:
        ignoreOp = true;
        handleDown();
        break;
    }

    SmartDashboard.putBoolean("Ignore Operator", ignoreOp);
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
    double rightAxisValue = -RobotContainer.driveStick.getRightX();
    if (Math.abs(rightAxisValue) > 0.03) { // Apply a deadband
        rightSpeed = Constants.slowMode
                    ? Constants.slowModeMultipler * rightAxisValue
                    : rightAxisValue;
    } else {
        rightSpeed = 0;
    }

    // Set the motor speeds
    double scalingConstant = 1;
    
    leftSpeed = Math.signum(leftSpeed) * (Math.log(1 + scalingConstant * Math.abs(leftSpeed)) / Math.log(1 + scalingConstant));
    // rightSpeed = Math.signum(rightSpeed) * (Math.log(1 + scalingConstant * Math.abs(rightSpeed)) / Math.log(1 + scalingConstant));
    double rightDriveTrainSpeed = leftSpeed;
    double leftDriveTrainSpeed = leftSpeed;
    if(rightSpeed > 0) {
      if ((leftDriveTrainSpeed + rightSpeed) > 1) {
        leftDriveTrainSpeed -= rightSpeed;
      } else if((rightDriveTrainSpeed - rightSpeed) > 1) {
        rightDriveTrainSpeed += (rightSpeed);
      } else {
        leftDriveTrainSpeed -= (rightSpeed / 2);
        rightDriveTrainSpeed += (rightSpeed / 2);
      }
    } else {
      if ((rightDriveTrainSpeed + rightSpeed) < -1) {
        rightDriveTrainSpeed += rightSpeed;
      } else if((leftDriveTrainSpeed - rightSpeed) > 1) { 
        leftDriveTrainSpeed -= rightSpeed;
      } else {
        leftDriveTrainSpeed -= (rightSpeed / 2);
        rightDriveTrainSpeed += (rightSpeed / 2);
      }
    }

    // RobotContainer.drivetrain.setRightSideMotorSpeed(rightSpeed);
    // RobotContainer.drivetrain.arcadeDrive(leftSpeed, rightSpeed);
    if(!ignoreDrive) {
      if(slowModeMultiplier) {
      RobotContainer.drivetrain.setAllMotorsSpeed(0.2* leftDriveTrainSpeed, 0.2 * rightDriveTrainSpeed);
      } else {
        RobotContainer.drivetrain.setAllMotorsSpeed(leftDriveTrainSpeed, rightDriveTrainSpeed);
      }
    }
}

  private void handleRightTriggerPressed(double speed) {
    RobotContainer.shooter.shoot_that_fucker(speed);
    try {
      Thread.sleep(250);
    } catch(InterruptedException e) {
      e.printStackTrace();
    }
    ignoreOp = false;
    centerMultiplier = 1;
  }

  private void triggerPressed(double speed) {
    RobotContainer.shooter.reverse(speed);
  }

  private void intakeStart() {
    RobotContainer.shooter.intakeStart();
  }
  private void intakeStop() {
    RobotContainer.shooter.intakeStop();
  }
  private void handleUp() {
    RobotContainer.elevator.moveRotation(1.0);
  }

  private void handleDown() {
    RobotContainer.elevator.moveRotation(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.drivetrain.setLeftSideMotorSpeed(0);
    // RobotContainer.drivetrain.setRightSideMotorSpeed(0);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
