// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.Constants.OIConstants;

public class Pivot extends SubsystemBase {

  // public static XboxController m_operatorController = new
  // XboxController(OIConstants.kOperatorControllerPort);

  private static CANSparkMax armLift;
  public SparkPIDController arm_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private static final int ROBORIO_DIO0 = 0;
  private static final int ROBORIO_DIO1 = 1;

  // the physical micro switches are Normally Open (false when not pressed, true
  // when pressed)
  // the magnetic switch is Normally Closed (false when near magnet, true when
  // apart from magnet)
  private static DigitalInput topLimitSwitch = new DigitalInput(ROBORIO_DIO0);
  private static DigitalInput bottomLimitSwitch = new DigitalInput(ROBORIO_DIO1);

  public static PIDController pid = new PIDController(1.8, 0, 0);
  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder()
   * method on an existing CANSparkMax object. If using a REV Through Bore
   * Encoder, the type should be set to quadrature and the counts per
   * revolution set to 8192
   */
  private SparkAbsoluteEncoder arm_absoluteEncoder;
  private double currentPosition;
  // private double targetPosition;
  // private double armSpeed;
  // private double speedMultiplier = 0.7;
  // private double speedOffset = 0.1;
  // private double positionTolerance = 0.025;
  // private Boolean holdingUpArm = false;
  // private double holdingPower = 0;
  // private boolean goingDown = false;

  // private double actualEncoderSum;
  // private double lastEncoder;

  /** Creates a new Arm. */
  public Pivot() {
    // initialize motor
    armLift = new CANSparkMax(17, MotorType.kBrushless);
    armLift.setInverted(false);
    arm_absoluteEncoder = armLift.getAbsoluteEncoder(Type.kDutyCycle);
    arm_absoluteEncoder.setInverted(true);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    arm_pidController = armLift.getPIDController();

    /**
     * By default, the PID controller will use the Hall sensor from a NEO for its
     * feedback device. Instead, we can set the feedback device to the alternate
     * encoder object
     */
    arm_pidController.setFeedbackDevice(arm_absoluteEncoder);

    /**
     * From here on out, code looks exactly like running PID control with the
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */

    // PID coefficients
    kP = 1.4;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    // arm_pidController.setP(kP);
    // arm_pidController.setI(kI);
    // arm_pidController.setD(kD);
    // arm_pidController.setIZone(kIz);
    // arm_pidController.setFF(kFF);
    // arm_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0.001);
    // pid.setSetpoint(0.8);
    // armLift.set(0);
    // setArmPosition(.95);

    // actualEncoderSum = arm_absoluteEncoder.getPosition(); // initalizes the
    // encoder value
  }

  @Override
  public void periodic() {

    moveArm(RobotContainer.m_operatorController.getLeftY());
    // SmartDashboard.putNumber("Joystick Value", RobotContainer.m_operatorController.getLeftY());
    // if (currentPosition <= targetPosition + positionTolerance
    // && currentPosition >= targetPosition - positionTolerance) {
    // armLift.set(0);
    // armLift.setIdleMode(IdleMode.kBrake);
    // }
    // if (goingDown != true) {
    // if (speakerLimitSwitch.get() == true) {
    // armLift.set(0);
    // armLift.setIdleMode(IdleMode.kBrake);
    // }
    // }
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0.001);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if ((p != kP)) {
    //   arm_pidController.setP(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   arm_pidController.setI(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   arm_pidController.setD(d);
    //   kD = d;
    // }
    // if ((iz != kIz)) {
    //   arm_pidController.setIZone(iz);
    //   kIz = iz;
    // }
    // if ((ff != kFF)) {
    //   arm_pidController.setFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    //   arm_pidController.setOutputRange(min, max);
    //   kMinOutput = min;
    //   kMaxOutput = max;
    // }

    currentPosition = arm_absoluteEncoder.getPosition();

    // armLift.set(0);
    SmartDashboard.putNumber("Arm Speed", armLift.get());
    SmartDashboard.putBoolean("Top Limitswitch", topLimitSwitch.get());
    SmartDashboard.putBoolean("Bottom Limitswitch", bottomLimitSwitch.get());

    // SPEED LOGIC
    // armSpeed = ((speedMultiplier) * Math.abs(targetPosition - currentPosition)) +
    // speedOffset; // arm motor speed calc,
    // Min Speed = 0.1

    // if (armSpeed > 0.7) {
    // armSpeed = 0.7; // motor speed max speed cap: 0.5
    // }

    // POSITION LOGIC
    // THIS MOVES THE SHOOTER ARM DOWN
    // if (currentPosition < targetPosition - positionTolerance) {
    // arm_pidController.setReference(targetPosition,
    // CANSparkMax.ControlType.kPosition);

    // only increase postion if the limit switch is not hit
    // if (ampLimitSwitch.get() == true) {
    // armLift.set(armSpeed);
    // } else {
    // armLift.set(holdingPower);
    // }
    // THIS MOVES THE SHOOTER ARM DOWN
    // } else if (currentPosition > targetPosition + positionTolerance) {
    // arm_pidController.setReference(targetPosition,
    // CANSparkMax.ControlType.kPosition);

    // only increase postion if the limit switch is not hit
    // if (speakerLimitSwitch.get() == true) {
    // if (speakerLimitSwitch.get() == false) {
    // armLift.set(-armSpeed);
    // } else {
    // armLift.set(0);
    // armLift.setIdleMode(IdleMode.kBrake);
    // }
    // } else {
    // armLift.set(0);
    // }
    // } else {
    // arm_pidController.setReference(targetPosition,
    // CANSparkMax.ControlType.kPosition);
    // if(holdingUpArm) {
    // armLift.set(holdingPower);
    // } else {
    // armLift.set(0);
    // }
    // }

    // if (position > 0.5) position -= 1;

    // double power = pid.calculate(position);
    // if (power > 1)
    // power = 1;
    // if (power < -1)
    // power = -1;
    // armLift.set(power);

    // SmartDashboard.putNumber("lift power", power);

    /**
     * 
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * com.revrobotics.CANSparkMax.ControlType.kPosition
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     */

    // arm_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    // SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("Encoder", currentPosition);
    // SmartDashboard.putNumber("Target Position", targetPosition);
    // SmartDashboard.putBoolean("Going Down", goingDown);

  }

  public static void moveArm(double joystickValue) {

    if (joystickValue >= 0.1) { // THIS IS TRUE BECAUSE THE MAGNET IS REVERSE LOGIC
      if (bottomLimitSwitch.get() == true) {
     //   System.out.println("pivot going down");
        armLift.setIdleMode(IdleMode.kCoast);
        armLift.set(joystickValue * 0.6);
      } else {
    //  System.out.println("pivot stop");
      armLift.set(0);
      armLift.setIdleMode(IdleMode.kBrake);
    }
    } else if (joystickValue < -0.1) {
      if (topLimitSwitch.get() == true) {
   //     System.out.println("pivot going up");
        armLift.setIdleMode(IdleMode.kCoast);
        armLift.set(joystickValue * 0.6);
      } else {
   //   System.out.println("pivot stop");
      armLift.set(0);
      armLift.setIdleMode(IdleMode.kBrake);
    }
    } else {
     // System.out.println("pivot stop");
      armLift.set(0);
      armLift.setIdleMode(IdleMode.kBrake);
    }
  }

  public void armBrake() {
    armLift.setIdleMode(IdleMode.kBrake);
    armLift.set(0.0);
  }
}
