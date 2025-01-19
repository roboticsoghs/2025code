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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final SparkMax motor;

    // motor configs
    private final SparkMaxConfig config;

    // motor PID controllers
    private final SparkClosedLoopController PID;

    // motor encoders
    private final RelativeEncoder Encoder;
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

    // elevator
    private double restingPosition;

    public final double allowedError = 0.05;

    public Shooter() {

        // init motors
        motor = new SparkMax(Constants.shooterPort, MotorType.kBrushless);

        // init config
        config = new SparkMaxConfig();

        // init PID controllers (closed loop)
        PID = motor.getClosedLoopController();

        Encoder = motor.getEncoder();

        // set idle mode for motors
        config.idleMode(IdleMode.kBrake);

        // set voltage compensation
        config.voltageCompensation(12.0);

        // Set current limits
        config.smartCurrentLimit(40);

        configurePIDControllers();

        // restore factory defaults and set new configs for each motor
        config.signals.primaryEncoderPositionPeriodMs(5);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configurePIDControllers() {
        // Apply PID constants to the PID controllers
        // left motor
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD);
        config.closedLoop.maxMotion.maxAcceleration(maxAccel);
        config.closedLoop.maxMotion.maxVelocity(maxVel);
        config.closedLoop.velocityFF(SmartVelocityFF);
        config.closedLoop.maxMotion.allowedClosedLoopError(allowedError);
    }

    /**
     * @return position in ticks
     */
    public double getEncoder() {
        return Encoder.getPosition() * 42;
    }

    public void shoot_that_fucker() {
        // Shoot coral at 20% speed
        PID.setReference(0.2, ControlType.kDutyCycle);
    }
}