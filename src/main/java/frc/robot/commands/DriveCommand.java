package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends Command {
    private int leftSpeed;
    private int rightSpeed;
    private boolean finished = false;

    public DriveCommand() {
        this.finished = false;
        addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void execute() {
        // Get joystick values from Xbox controller
        
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        RobotContainer.drivetrain.setLeftSideMotorSpeed(0);
        RobotContainer.drivetrain.setRightSideMotorSpeed(0);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        // This command should never end on its own
        return false;
    }
}
