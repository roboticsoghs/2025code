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

  public void makeParallel() {
    if(isAprilTag()) {
      double angle = getPitch();
      double distToMove = Math.sin(Math.toRadians(angle)) * Constants.DriveTrainGearRatio;
      if(angle > 0) {
        Robotcontainer.drivetrain.setRightSideMotorsPosition(-distToMove);
      } else {
        Robotcontainer.drivetrain.setRightSideMotorsPosition(distToMove);
      }
    }
  }

  /**
   * Auto aligns the robot to the left reef pole
   */
  public void alignLeftSide() {
    if(isApriltag()) {
      double distToMove = Constants.leftAlignReef - (getX() / 39.37);
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      makeParallel();
      SmartDashboard.putNumber("rotations to move", rotations);
      RobotContainer.drivetrain.setAllMotorsPosition(Robotcontainer.drivetrain.getleftFrontEncoder() + rotations, Robotcontainer.drivetrain.getrightFrontEncoder() + rotations);
    }
  }

  /**
   * Auto aligns the robot to the right reef pole
   */
  public void alignRightSide() {
    if(isApriltag()) {
      double distToMove = Constants.rightAlignReef - (getX() / 39.37);
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      makeParallel();
      SmartDashboard.putNumber("rotations to move", rotations);
      RobotContainer.drivetrain.setAllMotorsPosition(Robotcontainer.drivetrain.getleftFrontEncoder() + rotations, Robotcontainer.drivetrain.getrightFrontEncoder() + rotations);
    }
  }

  /**
   * Auto aligns the robot between the reef poles
   */
  public void alignCenterSide() {
    if(isApriltag()) {
      double distToMove = Constants.reefCenter - (getX() / 39.37);
      double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
      RobotContainer.drivetrain.resetAllEncoders();
      makeParallel();
      SmartDashboard.putNumber("rotations to move", rotations);
      RobotContainer.drivetrain.setAllMotorsPosition(Robotcontainer.drivetrain.getleftFrontEncoder() + rotations, Robotcontainer.drivetrain.getrightFrontEncoder() + rotations);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
