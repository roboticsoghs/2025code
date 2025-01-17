package frc.robot.subsystems;

import java.util.function.IntPredicate;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
    // motors
    private SparkMax leftFrontMotor;
    private SparkMax leftRearMotor;
    private SparkMax rightFrontMotor;
    private SparkMax rightRearMotor;

    // motor configs
    private SparkMaxConfig leftFrontConfig;
    private SparkMaxConfig leftRearConfig;
    private SparkMaxConfig rightFrontConfig;
    private SparkMaxConfig rightRearConfig;

    // motor PID controllers
    private final SparkClosedLoopController leftFrontPID;
    private final SparkClosedLoopController leftRearPID;
    private final SparkClosedLoopController rightFrontPID;
    private final SparkClosedLoopController rightRearPID;

    // motor encoders
    private final RelativeEncoder leftFrontEncoder;
    private final RelativeEncoder leftRearEncoder;
    private final RelativeEncoder rightFrontEncoder;
    private final RelativeEncoder rightRearEncoder;

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
    private int SmartVelocityID = 1;
    private final int maxVel = 4075;

    public final double allowedError = 0.05;

    private final DifferentialDrive differentialDrive;

    public DriveSubsystem() {
        // init motors
        leftFrontMotor = new SparkMax(Constants.leftFrontMotorPort, MotorType.kBrushless);
        leftRearMotor = new SparkMax(Constants.leftBackMotorPort, MotorType.kBrushless);
        rightFrontMotor = new SparkMax(Constants.rightFrontMotorPort, MotorType.kBrushless);
        rightRearMotor = new SparkMax(Constants.rightBackMotorPort, MotorType.kBrushless);

        // init config
        leftFrontConfig = new SparkMaxConfig();
        leftRearConfig = new SparkMaxConfig();
        rightFrontConfig = new SparkMaxConfig();
        rightRearConfig = new SparkMaxConfig();

        // Configuring the rear motors to follow the front motors
        leftRearConfig.follow(leftFrontMotor);
        rightRearConfig.follow(rightFrontMotor);

        // init PID controllers (closed loop)
        leftFrontPID = leftFrontMotor.getClosedLoopController();
        leftRearPID = leftRearMotor.getClosedLoopController();
        rightFrontPID = rightFrontMotor.getClosedLoopController();
        rightRearPID = rightRearMotor.getClosedLoopController();

        // init encoders
        leftFrontEncoder = leftFrontMotor.getEncoder();
        leftRearEncoder = leftRearMotor.getEncoder();
        rightFrontEncoder = rightFrontMotor.getEncoder();
        rightRearEncoder = rightRearMotor.getEncoder();

        // configurePIDControllers();

        // set idle mode for motors
        leftFrontConfig.idleMode(IdleMode.kBrake);
        leftRearConfig.idleMode(IdleMode.kBrake);
        rightFrontConfig.idleMode(IdleMode.kBrake);
        rightRearConfig.idleMode(IdleMode.kBrake);
        
        leftFrontConfig.inverted(false);
        // leftRear.setInverted(false);
        rightFrontConfig.inverted(true);
        // rightRear.setInverted(true);

        // Enable voltage compensation
        // leftFront.enableVoltageCompensation(12.0);
        // leftRear.enableVoltageCompensation(12.0);
        // rightFront.enableVoltageCompensation(12.0);
        // rightRear.enableVoltageCompensation(12.0);
        leftFrontConfig.voltageCompensation(12.0);
        leftRearConfig.voltageCompensation(12.0);
        rightFrontConfig.voltageCompensation(12.0);
        rightRearConfig.voltageCompensation(12.0);

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

        // restore factory defaults and set new configs for each motor
        leftFrontConfig.signals.primaryEncoderPositionPeriodMs(5);
        leftFrontMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftRearConfig.signals.primaryEncoderPositionPeriodMs(5);
        leftRearMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightFrontConfig.signals.primaryEncoderPositionPeriodMs(5);
        rightFrontMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightRearConfig.signals.primaryEncoderPositionPeriodMs(5);
        rightRearMotor.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // differential drive init
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
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
        // leftFrontPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);

        // left rear motor
        leftRearPID.setP(SmartVelocityP, SmartVelocityID);
        leftRearPID.setI(SmartVelocityI, SmartVelocityID);
        leftRearPID.setD(SmartVelocityD, SmartVelocityID);
        leftRearPID.setFF(SmartVelocityFF, SmartVelocityID);
        leftRearPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        leftRearPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        leftRearPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        // leftRearPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);

        // right front motor
        rightFrontPID.setP(SmartVelocityP, SmartVelocityID);
        rightFrontPID.setI(SmartVelocityI, SmartVelocityID);
        rightFrontPID.setD(SmartVelocityD, SmartVelocityID);
        rightFrontPID.setFF(SmartVelocityFF, SmartVelocityID);
        rightFrontPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        rightFrontPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        rightFrontPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        // rightFrontPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);

        // right rear motor
        rightRearPID.setP(SmartVelocityP, SmartVelocityID);
        rightRearPID.setI(SmartVelocityI, SmartVelocityID);
        rightRearPID.setD(SmartVelocityD, SmartVelocityID);
        rightRearPID.setFF(SmartVelocityFF, SmartVelocityID);
        rightRearPID.setOutputRange(MinOutput, MaxOutput, SmartVelocityID);

        rightRearPID.setSmartMotionMaxAccel(maxAccel, SmartVelocityID);
        rightRearPID.setSmartMotionMaxVelocity(maxVel, SmartVelocityID);
        // rightRearPID.setSmartMotionMinOutputVelocity(minVel, SmartVelocityID);
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

    public void setLeftSideMotorSpeed(double input) {
        double speed = input * maxVel;
        System.out.println("Left value" + speed);
        leftFrontPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
        leftRearPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
    }

    public void setRightSideMotorSpeed(double input) {
        double speed = input * maxVel;
        System.out.print(" Right value" + speed);
        rightFrontPID.setReference(speed, ControlType.kSmartVelocity, SmartVelocityID);
        rightRearPID.setReference(speed,ControlType.kSmartVelocity, SmartVelocityID);
    }


    public void setAllMotorsSpeed(double speed) {
        setLeftSideMotorSpeed(speed);
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