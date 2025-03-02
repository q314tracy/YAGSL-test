// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsys;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**Subsys built to test YAGSL functionality.*/
public class SwerveMain extends SubsystemBase {

  //class-level declarations
  private final double k_maximumSpeed;
  private final File swerve_directory;
  private SwerveDrive swerveDrive;

  //robot config for PP
  private RobotConfig pp_robotConfig;

  public SwerveMain() {

    //max speed variable
    k_maximumSpeed = Units.feetToMeters(4.5);

    //swerve configuration directory
    swerve_directory = new File(Filesystem.getDeployDirectory(),"swerve");

    //handle IO exception if swerve directory is empty
    //ALSO CHECK JSON FILES THAT CONFIG IS CORRECT, SOME DATA IS STILL BLANK AND NODE IDS NEED ADJUSTED DUMMY
    try {
      swerveDrive = new SwerveParser(swerve_directory).createSwerveDrive(k_maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace(); // Prints the error to the console for debugging
    }

    //autobuilder for PP
    //need to work on getting 2197's swerve subsytem calls swapped over to YAGSL calls for autobuilder to work, some fixed but not all

    //pose supplier ok
    //reset odometry ok
    //chassis speeds supplier probably needs composed from individual YAGSL calls, unsure if there is specific call for robot-relative speeds atm
    //drive method needs checked, must consume chassis speeds object
    //closed loop controller tuning?

    /**
    try {
      pp_robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    //configure autobuilder
    AutoBuilder.configure(
      swerveDrive.getPose(), // Robot pose supplier
      swerveDrive.resetOdometry(new Pose2d()), // Method to reset odometry (will be called if your auto has a starting pose)
      this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      pp_robotConfig, // The robot configuration
      () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      },
      this // Reference to this subsystem to set requirements
    );
    */
  }

  

 /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
