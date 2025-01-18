// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    // motor configs
    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    // motor PID controllers
    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController rightPID;

    // motor encoders
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    // SmartVelocity PID
    private final double SmartVelocityP = 0.0002;
    private final double SmartVelocityI = 0; // 0.00003
    private final double SmartVelocityD = 0;
    private final double SmartVelocityFF = 0;

    // max motor speed
    private final double MaxOutput = 1;
    // min motor speed
    private final double MinOutput = -1;

    // max motor acceleration
    private final double maxAccel = 1000000000;
    private final int SmartMotionID = 0;
    private int MaxMotionID = 1;
    private final int maxVel = 4075;

    public final double allowedError = 0.05;

    public ElevatorSubsystem() {
        // init motors
        leftMotor = new SparkMax(Constants.elevatorLeftMotorPort, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightMotorPort, MotorType.kBrushless);

        // init config
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        // Configuring the rear motors to follow the front motors
        leftConfig.follow(rightMotor);

        // init PID controllers (closed loop)
        leftPID = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        // init encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // set idle mode for motors
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftConfig.inverted(false);
        // leftRear.setInverted(false);
        rightConfig.inverted(true);
        // rightRear.setInverted(true);

        // set voltage compensation
        leftConfig.voltageCompensation(12.0);
        rightConfig.voltageCompensation(12.0);

        // Set current limits
        leftConfig.smartCurrentLimit(40);
        rightConfig.smartCurrentLimit(40);

        configurePIDControllers();

        // restore factory defaults and set new configs for each motor
        leftConfig.signals.primaryEncoderPositionPeriodMs(5);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightConfig.signals.primaryEncoderPositionPeriodMs(5);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configurePIDControllers() {
        // Apply PID constants to the PID controllers
        // left motor
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        leftConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        leftConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        leftConfig.closedLoop.velocityFF(SmartVelocityFF);
        leftConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);

        // right motor
        rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightConfig.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        rightConfig.closedLoop.maxMotion.maxAcceleration(maxAccel);
        rightConfig.closedLoop.maxMotion.maxVelocity(maxVel);
        rightConfig.closedLoop.velocityFF(SmartVelocityFF);
        rightConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
    }

    public double getleftEncoder() {
        return leftEncoder.getPosition();
    }


    public double getRightEncoder() {
        return rightEncoder.getPosition();
    }
}