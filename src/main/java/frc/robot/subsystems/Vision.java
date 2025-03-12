// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  private double x;
  private double y;
  private double z;
  private double pitch;
  private double yaw;
  private double roll;
  private double[] camera = new double[6];

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry cameraPose = table.getEntry("targetpose_cameraspace");

  public Vision() {
    System.out.println("We are cooked!");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    double[] defaultValue = {0,0,0,0,0,0}; // default value required by getDoubleArray
    camera = cameraPose.getDoubleArray(defaultValue);

    // limelight 3D offsets relative to camera
    x = camera[0];
    y = camera[1];
    z = camera[2];
    pitch = camera[3];
    yaw = camera[4];
    roll = camera[5];

    // angleX = ca

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightZ", z);

    SmartDashboard.putNumber("Limelight Pitch", pitch);
    SmartDashboard.putNumber("Limelight Yaw", yaw);
    SmartDashboard.putNumber("Limelight Roll", roll);

    // output if the limelight sees an AprilTag
    SmartDashboard.putBoolean("AprilTag Seen", isApriltag());
  }

  /**
   *
   * @return Whether the Limelight detects an AprilTag
   */
  public boolean isApriltag() {
    return x != 0 && z != 0;
  }

  /**
   *
   * @return The x offset of the AprilTag relative to the Limelight (in meters)
   */
  public double getX() {
    return x;
  }

  /**
   *
   * @return The z offset of the AprilTag relative to the Limelight (in meters)
   */
  public double getZ() {
    return z;
  }

  public double getPitch() {
    return pitch;
  }

  public double getYaw() {
    return yaw;
  }

  public double getRoll() {
    return roll;
  }

  public void makeParallel(double angle) {
    double distToMove = Math.sin(Math.toRadians(angle)) * (Constants.DriveTrainGearRatio);
    SmartDashboard.putNumber("rotations to rotate", distToMove);
    if(angle < 0 && Math.abs(distToMove) > 1.0) {
      distToMove *= 0.7;
    }
    if(angle > 0 && Math.abs(distToMove) > 1.0) {
      distToMove *= 0.85;
    }
    RobotContainer.drivetrain.setRightSideMotorsPosition(-distToMove);
    RobotContainer.drivetrain.setLeftSideMotorsPosition(distToMove);
    try {
      Thread.sleep(1000);
    } catch(InterruptedException e) {
        e.printStackTrace();
    }
  }

  /**
   * Auto aligns the robot to the left reef pole
   */
  public void alignToReef(double offsetHeight) {
    if(isApriltag()) {
      double distToMove = offsetHeight - (getX() / 39.37);
      double yawVal = getYaw();
      double zVal = getZ();
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      if(Math.abs(yawVal) > 5) {
        makeParallel(yawVal);
      }
      SmartDashboard.putNumber("rotations to move", rotations);
      RobotContainer.drivetrain.resetAllEncoders();
      RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);
      adjustElevatorHeight(zVal);
    }
  }

  /**
   * Auto aligns the robot to the right reef pole
   */
  // public void alignRightSide() {
  //   if(isApriltag()) {
  //     double distToMove = Constants.rightAlignReef - (getX() / 39.37);
  //     double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
  //     RobotContainer.drivetrain.resetAllEncoders();
  //     makeParallel();
  //     SmartDashboard.putNumber("rotations to move", rotations);
  //     RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getleftFrontEncoder() + rotations, RobotContainer.drivetrain.getRightFrontEncoder() + rotations);
  //   }
  // }

  /**
   * Auto aligns the robot between the reef poles
   */
  // public void alignCenterSide() {
  //   if(isApriltag()) {
  //     double distToMove = Constants.reefCenter - (getX() / 39.37);
  //     double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
  //     RobotContainer.drivetrain.resetAllEncoders();
  //     makeParallel();
  //     SmartDashboard.putNumber("rotations to move", rotations);
  //     RobotContainer.drivetrain.setAllMotorsPosition(RobotContainer.drivetrain.getleftFrontEncoder() + rotations, RobotContainer.drivetrain.getRightFrontEncoder() + rotations);
  //     adjustElevatorHeight();
  //   }
  // }

  /**
   * Adjust elevator height based on distance from the reef
   */
  private void adjustElevatorHeight(double zVal) {
    if (zVal <= 0.05) {
      // move down 1 rotation when within 5cm of the reef
      RobotContainer.elevator.moveRotation(-1);
    } else if (zVal > 0.10) {
      // move up 1 rotation when further than 10cm from the reef
      RobotContainer.elevator.moveRotation(1);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
