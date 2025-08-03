package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class AutoCommand {
    public static Command Auto1_CENTER() {
        return Commands.sequence(
            Commands.runOnce(() -> RobotContainer.drivetrain.setAllMotorsSpeed(0.5, 0.5), RobotContainer.drivetrain),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> RobotContainer.drivetrain.stopMotor(), RobotContainer.drivetrain)
        );
    }

    public static Command Auto2_RIGHT() {
        return Commands.sequence(
            Commands.run(() -> RobotContainer.drivetrain.setAllMotorsSpeed(0.3, 0.3), RobotContainer.drivetrain, RobotContainer.visionSystem)
                .until(AutoCommand::isValidAprilTag),
            Commands.runOnce(() -> RobotContainer.drivetrain.stopMotor(), RobotContainer.drivetrain),
            Commands.runOnce(() -> RobotContainer.visionSystem.makeParallel(-0.3, true), RobotContainer.visionSystem),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0), RobotContainer.elevator),
            Commands.runOnce(() -> RobotContainer.elevator.moveRotation(2), RobotContainer.elevator),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> RobotContainer.shooter.outtake(0.3), RobotContainer.shooter)
        );
    }

    public static Command Auto3_LEFT() {
        return Commands.sequence(
            Commands.run(() -> RobotContainer.drivetrain.setAllMotorsSpeed(-0.3, -0.3), RobotContainer.drivetrain, RobotContainer.visionSystem)
                .until(AutoCommand::isValidAprilTag),
            Commands.runOnce(() -> RobotContainer.drivetrain.stopMotor(), RobotContainer.drivetrain),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> RobotContainer.drivetrain.setAllMotorsSpeed(0.1, -0.1), RobotContainer.visionSystem),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> RobotContainer.drivetrain.stopMotor(), RobotContainer.drivetrain),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> RobotContainer.elevator.moveRotation(15), RobotContainer.elevator),            
            Commands.waitSeconds(0.7),
            Commands.runOnce(() -> RobotContainer.shooter.outtake(0.3), RobotContainer.shooter)
        );
    }

    public static boolean isValidAprilTag() {
        return RobotContainer.visionSystem.isApriltag() ;
            // && (RobotContainer.visionSystem.getId() == 6 || RobotContainer.visionSystem.getId() == 19 || RobotContainer.visionSystem.getId() == 17 || RobotContainer.visionSystem.getId() == 8);
    }
}

// OLD AUTON

// public class AutoCommand extends Command {
//     private boolean finished = false;
//     private boolean detectedTag = false;
//     private boolean linedUp = false;
//     private boolean movedBack = false;
//     private double lineUpWithAprilTag = 0;
//     double finalPosition = 0;
//     private double position = 0;

//     public AutoCommand() {
//         addRequirements(RobotContainer.drivetrain, RobotContainer.visionSystem);
//         RobotContainer.drivetrain.resetAllEncoders();

//         // return Commands.sequence(
//         //     Commands.run(() -> RobotContainer.drivetrain.setAllMotorsSpeed(0.5, 0.5))
//         // );
//     }

//     @Override
//     public void execute() {
//         SmartDashboard.putNumber("auto pos: ", position);
//         SmartDashboard.putBoolean("auto finished", finished);

//         // select auto
//         // Auto1_LEFTSTART();
//         // Auto2_RIGHTSTART();
//         Auto3_CENTERSTART();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         RobotContainer.drivetrain.stopMotor(); // Stop motors
//     }

//     @Override
//     public boolean isFinished() {
//         return finished;
//     }

//     public void forward() {
//         RobotContainer.drivetrain.setAllMotorsPosition(position + 0.001, position + 0.001);
//         position++;
//     }

//     public void back() {
//         RobotContainer.drivetrain.setAllMotorsSpeed(position - 0.001, position - 0.001);
//         position--;
//     }


//     private void Auto1_LEFTSTART() {
//         if (RobotContainer.visionSystem.isApriltag()
            // && (RobotContainer.visionSystem.getId() == 6 || RobotContainer.visionSystem.getId() == 19 || RobotContainer.visionSystem.getId() == 17 || RobotContainer.visionSystem.getId() == 8)
//         ) {
//             RobotContainer.drivetrain.stopMotor();
//             detectedTag = true;

//             RobotContainer.visionSystem.makeParallel(-0.3, true);
//             try {
//                 Thread.sleep(1500);
//             } catch(InterruptedException e) {
//                 e.printStackTrace();
//             }
            
//             RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0);
//             RobotContainer.elevator.moveRotation(2);
//             try {
//                 Thread.sleep(500);
//             } catch(InterruptedException e) {
//                 e.printStackTrace();
//             }
//             RobotContainer.shooter.shoot_that_fucker(0.25);
//             try {
//                 Thread.sleep(500);
//             } catch(InterruptedException e) {
//                 e.printStackTrace();
//             }
//         } else {
//             SmartDashboard.putBoolean("AprilTag Seen", false);

//             if (!detectedTag) {
//                 // Keep driving forward until it sees a tag
//                 back();
//             }
//         }
//     }

//     private void Auto2_RIGHTSTART() {
//         if (RobotContainer.visionSystem.isApriltag()
//             && (RobotContainer.visionSystem.getId() == 6 || RobotContainer.visionSystem.getId() == 19 || RobotContainer.visionSystem.getId() == 17 || RobotContainer.visionSystem.getId() == 8)
//         ) {
//             RobotContainer.drivetrain.stopMotor();
//             detectedTag = true;

//             RobotContainer.visionSystem.makeParallel(-0.3, true);
//             try {
//                 Thread.sleep(1500);
//             } catch(InterruptedException e) {
//                 e.printStackTrace();
//             }
            
//             RobotContainer.elevator.setPosition(ElevatorPosition.LEVEL_0);
//             RobotContainer.elevator.moveRotation(2);
//             try {
//                 Thread.sleep(500);
//             } catch(InterruptedException e) {
//                 e.printStackTrace();
//             }
//             RobotContainer.shooter.shoot_that_fucker(0.25);
//             try {
//                 Thread.sleep(500);
//             } catch(InterruptedException e) {
//                 e.printStackTrace();
//             }
//         } else {
//             SmartDashboard.putBoolean("AprilTag Seen", false);

//             if (!detectedTag) {
//                 // Keep driving forward until it sees a tag
//                 forward();
//             }
//         }
//     }

//     private void Auto3_CENTERSTART() {
//         RobotContainer.drivetrain.setAllMotorsPosition(25, 25);
//     }
// }
