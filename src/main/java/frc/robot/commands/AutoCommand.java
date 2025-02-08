package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoCommand extends Command {
    private int leftSpeed;
    private int rightSpeed;
    private boolean finished = false;

    public AutoCommand() {
        this.finished = false;
        // addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void execute() {
        // Get joystick values from Xbox controller
        // RobotContainer.drivetrain.setRightSideMotorSpeed(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        // RobotContainer.drivetrain.setLeftSideMotorSpeed(0);
        // RobotContainer.drivetrain.setRightSideMotorSpeed(0);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        // This command should never end on its own
        return false;
    }
}
