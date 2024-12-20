// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;

// public class Joysticks extends SubsystemBase {
//   public static double joystickLeftYValue = RobotContainer.m_operatorController.getLeftY();
//   public static double joystickRightYValue = RobotContainer.m_operatorController.getLeftY();
//   /** Creates a new Joysticks. */
//   public Joysticks() {

//   }

//   public void CheckForInput() {
    
//   }

//   @Override
//   public void periodic() {
//   joystickLeftYValue = RobotContainer.m_operatorController.getLeftY();
//   SmartDashboard.putNumber("JoystickValue", joystickLeftYValue);
//   joystickRightYValue = RobotContainer.m_operatorController.getRightY();
//   SmartDashboard.putNumber("JoystickValue", joystickRightYValue);
//   if (joystickLeftYValue > .75) {
//     Climber.LeftClimb(.2);
//     Climber.brakeCheck = true;
//   } else if (joystickLeftYValue < .25) {
//     Climber.LeftClimb(-.2);
//     Climber.brakeCheck = true;
//   }
//   if (joystickRightYValue > .75) {
//     Climber.RightClimb(.2);
//     Climber.brakeCheck = true;
//   } else if (joystickRightYValue < .25) {
//     Climber.RightClimb(-.2);
//     Climber.brakeCheck = true;
//   }
//     // This method will be called once per scheduler run
//   }
// }
