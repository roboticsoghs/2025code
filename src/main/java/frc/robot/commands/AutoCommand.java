package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class AutoCommand extends Command {
    private boolean finished = false;
    private boolean detectedTag = false;
    private boolean linedUp = false;
    private boolean movedBack = false;
    private double lineUpWithAprilTag = 0;
    double finalPosition = 0;
    private double position = 0;

    public AutoCommand() {
        addRequirements(RobotContainer.drivetrain, RobotContainer.visionSystem);
        RobotContainer.drivetrain.resetAllEncoders();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("auto pos: ", position);
        SmartDashboard.putBoolean("auto finished", finished);

        if (RobotContainer.visionSystem.isApriltag()) {
            SmartDashboard.putBoolean("AprilTag Seen", true);
            detectedTag = true;

            // Get offsets
            double offSetX = RobotContainer.visionSystem.getX();
            double offSetZ = RobotContainer.visionSystem.getZ();

            double distToMove = Constants.rightAlignReef - (offSetX * 39.37);
            double rotations = RobotContainer.drivetrain.lineartoRotations(distToMove);

            SmartDashboard.putNumber("amount to move: ", rotations);

            RobotContainer.drivetrain.stopMotor();
            RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getEncoderVal() + rotations, RobotContainer.drivetrain.getEncoderVal() + rotations);

            // RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0);
            // try {
            //     Thread.sleep(600);
            // } catch(InterruptedException e) {
            //     e.printStackTrace();
            // }
            // RobotContainer.shooter.shoot_that_fucker(0.5);
            // try {
            //     Thread.sleep(300);
            // } catch(InterruptedException e) {
            //     e.printStackTrace();
            // }
            // RobotContainer.shooter.shoot_that_fucker(0);

            // Stop the robot when the tag is detected
            // RobotContainer.drivetrain.stopMotor();
        } else {
            SmartDashboard.putBoolean("AprilTag Seen", false);

            if (!detectedTag) {
                // Keep driving forward until it sees a tag
                RobotContainer.drivetrain.setAllMotorsPosition(position - 1, position - 1);
                position--;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.setAllMotorsSpeed(0, 0); // Stop motors
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
