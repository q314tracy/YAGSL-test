// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsys.SwerveMain;

public class RobotContainer {

  //controller to drive
  private final CommandXboxController m_controller;

  //swervedrive
  private final SwerveMain sub_swervedrive;

  public RobotContainer() {

    //configure triggers
    configureBindings();

    //button pressy thing
    m_controller = new CommandXboxController(0);

    //swervy boi
    sub_swervedrive = new SwerveMain();

    //set default command for swervy boi
    sub_swervedrive.setDefaultCommand(new RunCommand(
      () -> 
        sub_swervedrive.driveCommand(
          () -> -m_controller.getLeftY(), 
          () -> -m_controller.getLeftX(), 
          () -> -m_controller.getRightY()),
          sub_swervedrive));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
