package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import java.util.List;

public class AutoCommand extends Command {
    private boolean finished = false;

    public AutoCommand() {
        // PathPlannerTrajectory trajectory = new PathPlannerAuto("Auto 1");

        this.finished = false;
        addRequirements(RobotContainer.position_drive, RobotContainer.elevator, RobotContainer.shooter, RobotContainer.visionSystem);
        // addRequirements(RobotContainer.drivetrain);
    }

    @Override
    public void execute() {
        // Get joystick values from Xbox controller
        RobotContainer.position_drive.setAllMotorsSpeed(30, 30);
        if(RobotContainer.visionSystem.isApriltag()) {
            double offSetX = RobotContainer.visionSystem.getX();
            // Let's say we want our x to be 0.05;
            double distanceToMove = offSetX - 0.05;
            if(distanceToMove > 0.02) {
                // Implement logic to move some amount of rotations
            }
        }
        RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0);
        RobotContainer.shooter.shoot_that_fucker(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        RobotContainer.position_drive.setAllMotorsSpeed(0, 0);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        // This command should never end on its own
        return false;
    }
}
