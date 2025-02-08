// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class GyroSubsystem extends SubsystemBase {
    private AnalogGyro gyro = new AnalogGyro(0); // DIO port

    public GyroSubsystem() {
        System.out.println("hi");
    }

    public void periodic() {
        // 
        double angle = Math.round(gyro.getAngle());
        String direction = "";
        switch (angle % 4) {
            case 0:
                direction = "North";
            case 1:
                direction = "West";
            case 2:
                direction = "South";
            case 3:
                direction = "East";
            default:
                break;
        }
        SmartDashboard.putNumber("angle: ", );
    }
}