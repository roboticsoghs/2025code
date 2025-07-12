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
  private long aprilTagId;
  private double[] camera = new double[6];

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry cameraPose = table.getEntry("targetpose_cameraspace");

  public Vision() {
    System.out.println("We are cooked!");
    aprilTagId = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    double[] defaultValue = {0,0,0,0,0,0}; // default value required by getDoubleArray
    camera = cameraPose.getDoubleArray(defaultValue);
    long defaultValueID = 0;
    aprilTagId = table.getEntry("tid").getInteger(defaultValueID);

    // limelight 3D offsets relative to camera
    x = camera[0]; // in meters
    y = camera[1]; // in meters
    z = camera[2]; // distance in meteres
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
    
    SmartDashboard.putNumber("AprilTag ID", aprilTagId);
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

  public double getId() {
    return aprilTagId;
  }

  // align robot to reef (angle is in degrees)
  public void makeParallel(double angleDegrees) {
    if (Math.abs(angleDegrees) < 2.0) return; // ignore small angles

    final double rotationsPerDegree = 0.05; // adjust value as needed
    double baseRotations = angleDegrees * rotationsPerDegree;

    // clamp wheel rotations between -4 and 4
    baseRotations = Math.max(-4.0, Math.min(4.0, baseRotations));

    // EXPIRAMENTAL
    if (Math.abs(angleDegrees) > 15.0) {
      baseRotations *= 0.85;
    } else if (Math.abs(angleDegrees) > 5.0) {
      baseRotations *= 0.9;
    }

    double leftRotations = baseRotations;
    double rightRotations = -baseRotations;

    RobotContainer.drivetrain.resetAllEncoders();
    RobotContainer.drivetrain.setAllMotorsPosition(leftRotations, rightRotations);

    // double distToMove = Math.sin(Math.toRadians(angle)) * (Constants.DriveTrainGearRatio);
    // SmartDashboard.putNumber("rotations to rotate", distToMove);
    // if(angle < 0 && Math.abs(distToMove) > 1.0) {
    //   distToMove *= 0.7;
    // }
    // if(angle > 0 && Math.abs(distToMove) > 1.0) {
    //   distToMove *= 0.85;
    // }

    // RobotContainer.drivetrain.setRightSideMotorsPosition(-distToMove);
    // RobotContainer.drivetrain.setLeftSideMotorsPosition(distToMove);
    // try {
    //   Thread.sleep(750);
    // } catch(InterruptedException e) {
    //     e.printStackTrace();
    // }
  }

  /**
   * Auto aligns the robot to the left reef pole
   */
  public void alignToReef(double offset) {
    if (!isApriltag()) return; // AprilTag not found yet

    double cameraYaw = getYaw();
    double z = getZ();

    makeParallel(cameraYaw); // doesn't make parallel when under 2 degrees error

    double xOffset = getX() / 39.37; // convert x offset from meters to inches
    double distanceToMove = offset - xOffset;
    double rotations = RobotContainer.drivetrain.lineartoRotations(distanceToMove);

    RobotContainer.drivetrain.resetAllEncoders();
    RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);

    adjustElevatorHeight(z);

    // if(isApriltag()) {
    //   // first make the robot parallel to the reef
    //   double yawVal = getYaw();
    //   if(Math.abs(yawVal) > 3) {
    //     makeParallel(yawVal);
    //   }
    //   // calculate distance needed to line up with reef
    //   double distToMove = offsetHeight - (getX() / 39.37);
    //   double zVal = getZ();
    //   double rotations = Math.round(RobotContainer.drivetrain.lineartoRotations(distToMove) * 100.0) / 100.0;
    //   RobotContainer.drivetrain.resetAllEncoders();
    //   SmartDashboard.putNumber("rotations to move", rotations);
    //   RobotContainer.drivetrain.resetAllEncoders();
    //   RobotContainer.drivetrain.setAllMotorsPosition(rotations, rotations);
    //   adjustElevatorHeight(zVal);
    // }
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
   * Adjust elevator height based on distance from the reef.
   * zVal is the distance to the detected AprilTag in meters
   */
  private void adjustElevatorHeight(double zVal) {
    final double idealDist = 0.10; // in meters
    final double deadzone = 0.02;
    final double maxError = 0.2;
    final double maxRotations = 4; // max of 4 rotations of adjustment
    
    double error = zVal - idealDist; // calculate driver error
    if (Math.abs(error) < deadzone) return; // no adjustment needed
    
    double normalizedError = Math.max(-1.0, Math.min(1.0, error / maxError));
    double rotationAmount = normalizedError * maxRotations;

    RobotContainer.elevator.moveRotation(rotationAmount);


    // if (zVal < 0.15) {
    //   // move down 1 rotation when within 15cm of the reef
    //   RobotContainer.elevator.moveRotation(-1);
    // } else if (zVal > 0.20) {
    //   // move up 1 rotation when further than 20cm from the reef
    //   RobotContainer.elevator.moveRotation(2);
    // } else if (zVal > 0.30) {
    //   // move up 4 rotation when further than 30cm from the reef
    //   RobotContainer.elevator.moveRotation(4);
    // } 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
