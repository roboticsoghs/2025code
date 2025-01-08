package frc.robot.subsystems;

import java.util.function.IntPredicate;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
    // motors
    private CANSparkMax leftFront;
    private CANSparkMax leftRear;
    private CANSparkMax rightFront;
    private CANSparkMax rightRear;

    // motor PID controllers
    private final SparkPIDController leftFrontPID;
    private final SparkPIDController leftRearPID;
    private final SparkPIDController rightFrontPID;
    private final SparkPIDController rightRearPID;

    // motor encoders
    private final RelativeEncoder leftFrontEncoder;
    private final RelativeEncoder leftRearEncoder;
    private final RelativeEncoder rightFrontEncoder;
    private final RelativeEncoder rightRearEncoder;

    // SmartVelocity PID
    private final double SmartVelocityP = 0.003;
    private final double SmartVelocityI = 0; // 0.00003
    private final double SmartVelocityD = 0;
    private final double SmartVelocityFF = 0;

    // max motor speed
    private final double MaxOutput = 1;
    // min motor speed
    private final double MinOutput = -1;

    // max motor acceleration
    private final double maxAccel = 25;
    private final int SmartMotionID = 0;
    private int SmartVelocityID = 1;
    private final int maxVel = 5;
    private final int minVel = 0;

    public final double allowedError = 0.05;

    private final DifferentialDrive differentialDrive;

    public DriveSubsystem() {
        // set values
        leftFront = new CANSparkMax(Constants.leftFrontMotorPort, MotorType.kBrushless);
        leftRear = new CANSparkMax(Constants.leftBackMotorPort, MotorType.kBrushless);
        rightFront = new CANSparkMax(Constants.rightFrontMotorPort, MotorType.kBrushless);
        rightRear = new CANSparkMax(Constants.rightBackMotorPort, MotorType.kBrushless);

        leftFront.restoreFactoryDefaults();
        leftRear.restoreFactoryDefaults();
        rightFront.restoreFactoryDefaults();
        rightRear.restoreFactoryDefaults();

        leftFrontPID = leftFront.getPIDController();
        leftRearPID = leftRear.getPIDController();
        rightFrontPID = rightFront.getPIDController();
        rightRearPID = rightRear.getPIDController();

        leftFrontEncoder = leftFront.getEncoder();
        leftRearEncoder = leftRear.getEncoder();
        rightFrontEncoder = rightFront.getEncoder();
        rightRearEncoder = rightRear.getEncoder();

        // Configuring the rear motors to follow the front motors
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        configurePIDControllers();

        leftFront.setIdleMode(IdleMode.kBrake);
        leftRear.setIdleMode(IdleMode.kBrake);
        rightFront.setIdleMode(IdleMode.kBrake);
        rightRear.setIdleMode(IdleMode.kBrake);

        leftFront.setInverted(false);
        leftRear.setInverted(false);
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        // Enable voltage compensation
        leftFront.enableVoltageCompensation(12.0);
        leftRear.enableVoltageCompensation(12.0);
        rightFront.enableVoltageCompensation(12.0);
        rightRear.enableVoltageCompensation(12.0);

        // Set current limits
        leftFront.setSmartCurrentLimit(40);
        leftRear.setSmartCurrentLimit(40);
        rightFront.setSmartCurrentLimit(40);
        rightRear.setSmartCurrentLimit(40);

        // Save configurations to flash
        leftFront.burnFlash();
        leftRear.burnFlash();
        rightFront.burnFlash();
        rightRear.burnFlash();

        differentialDrive = new DifferentialDrive(leftFront, rightFront);
    }

    private void configurePIDControllers() {
        // Apply PID constants to the PID controllers
        // left front motor
        leftFrontPID.setP(SmartVelocityP, SmartVelocityID);
        leftFrontPID.setI(SmartVelocityI, SmartVelocityID);
        leftFrontPID.setD(SmartVelocityD, SmartVelocityID);
        leftFrontPID.setFF(SmartVelocityFF, SmartVelocityID);
        leftFrontPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        leftFrontPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        leftFrontPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        leftFrontPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);

        // left rear motor
        leftRearPID.setP(SmartVelocityP, SmartVelocityID);
        leftRearPID.setI(SmartVelocityI, SmartVelocityID);
        leftRearPID.setD(SmartVelocityD, SmartVelocityID);
        leftRearPID.setFF(SmartVelocityFF, SmartVelocityID);
        leftRearPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        leftRearPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        leftRearPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        leftRearPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);

        // right front motor
        rightFrontPID.setP(SmartVelocityP, SmartVelocityID);
        rightFrontPID.setI(SmartVelocityI, SmartVelocityID);
        rightFrontPID.setD(SmartVelocityD, SmartVelocityID);
        rightFrontPID.setFF(SmartVelocityFF, SmartVelocityID);
        rightFrontPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        rightFrontPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        rightFrontPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        rightFrontPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);

        // right rear motor
        rightRearPID.setP(SmartVelocityP, SmartVelocityID);
        rightRearPID.setI(SmartVelocityI, SmartVelocityID);
        rightRearPID.setD(SmartVelocityD, SmartVelocityID);
        rightRearPID.setFF(SmartVelocityFF, SmartVelocityID);
        rightRearPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        rightRearPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        rightRearPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        rightRearPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);
    }

    public double getleftFrontEncoder() {
        return leftFrontEncoder.getPosition();
    }

    public double getleftRearEncoder() {
        return leftRearEncoder.getPosition();
    }

    public double getRightFrontEncoder() {
        return rightFrontEncoder.getPosition();
    }

    public double getRightRearEncoder() {
        return rightRearEncoder.getPosition();
    }

    public void setLeftSideMotorsSpeed(double input) {
        double speed = input * maxVel;
        if (Math.abs(input) < 0.05) {  // Dead zone for joystick
            speed = 0;  // Stop the motor
        }
        System.out.println("Left value" + speed);
        leftFrontPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
        leftRearPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
    }

    public void setRightSideMotorSpeed(double input) {
        double speed = input * maxVel;
        if (Math.abs(input) < 0.05) {  // Dead zone for joystick
            speed = 0;  // Stop the motor
        }
        System.out.print(" Right value" + speed);
        rightFrontPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
        rightRearPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
    }

    // Logarithmic scaling function
    private double scaleJoystickInput(double input) {
        // In meters per minute
        double linearVel = input * maxVel * 60;
        double rotationVel = linearVel / (2 * Math.PI * 0.07);
        double sign = Math.signum(rotationVel);
        double rpm = sign * Math.log1p(9 * Math.abs(rotationVel)) / Math.log1p(9);
        return rpm * leftFrontEncoder.getCountsPerRevolution();
    }

    public void setAllMotorsSpeed(double speed) {
        setLeftSideMotorsSpeed(speed);
        setRightSideMotorSpeed(speed);
    }

    public void setLeftSideMotorsPosition(double rotations) {
        leftFrontPID.setReference(rotations, ControlType.kDutyCycle);
        leftRearPID.setReference(rotations, ControlType.kDutyCycle);
    }

    public void setRightSideMotorsPosition(double rotations) {
        rightFrontPID.setReference(rotations, ControlType.kDutyCycle);
        rightRearPID.setReference(rotations, ControlType.kDutyCycle);
    }

    public void setAllMotorsPosition(double rotations) {
        setLeftSideMotorsPosition(rotations);
        setRightSideMotorsPosition(rotations);
    }

    public void resetLeftSideEncoders() {
        leftFrontEncoder.setPosition(0);
        leftRearEncoder.setPosition(0);
    }

    public void resetRightSideEncoders() {
        rightFrontEncoder.setPosition(0);
        rightRearEncoder.setPosition(0);
    }

    public void resetAllEncoders() {
        resetLeftSideEncoders();
        resetRightSideEncoders();
    }

    public void arcadeDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.arcadeDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {
        // Update odometry, smart dashboard, etc.
    }
}