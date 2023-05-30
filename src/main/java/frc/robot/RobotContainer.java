// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.FollowEventPath;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.FieldDriverStick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  FieldDriverStick m_driveStick = new FieldDriverStick(m_driverController);

  JoystickButton m_calibrateButton = new JoystickButton(m_driverController, 8);

  PathPlannerTrajectory path = PathPlanner.loadPath("New Path", 1, 1);
  

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    DriveConstants.kThetaController.enableContinuousInput(-180, 180);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_driveStick.getX(),
                    m_driveStick.getY() ,
                    m_driveStick.getZ(),
                    true),
            m_robotDrive));
        // new RunCommand(
        //     () ->
        //         m_robotDrive.drive(
        //             0,
        //             0.1,
        //             0,
        //             true),
        //     m_robotDrive));


        SmartDashboard.putNumber("Drive X P", DriveConstants.kXController.getP());
        SmartDashboard.putNumber("Drive Y P", DriveConstants.kYController.getP());
        SmartDashboard.putNumber("Drive X I", DriveConstants.kXController.getI());
        SmartDashboard.putNumber("Drive Y I", DriveConstants.kYController.getI());
        SmartDashboard.putNumber("Drive X D", DriveConstants.kXController.getD());
        SmartDashboard.putNumber("Drive Y D", DriveConstants.kYController.getD());
        
        SmartDashboard.putNumber("Theta P",DriveConstants.kThetaController.getP());
        SmartDashboard.putNumber("Theta I",DriveConstants.kThetaController.getI());
        SmartDashboard.putNumber("Theta D",DriveConstants.kThetaController.getD());
  }

  public void robotInit(){
    Constants.DriveConstants.kThetaController.enableContinuousInput(-180, 180);
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_calibrateButton.onTrue(new InstantCommand(() -> m_robotDrive.calibrate()));
    new JoystickButton(m_driverController,1).onTrue(new InstantCommand(() -> UpdateTrajectoryPIDValues()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(0.5, 0.5)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(1, 0, Rotation2d.fromDegrees(30)),
    //         AutoConstants.kMaxSpeedTrajectoryConfig);



    //FollowTrajectory trajectoryDrive = new FollowTrajectory(exampleTrajectory, m_robotDrive, true, true);
    FollowEventPath pathDrive = new FollowEventPath(path, m_robotDrive, true);
    
    // Run path following command, then stop at the end.
    return pathDrive;
   //m_robotDrive.setDisplayTrajectory(exampleTrajectory);
   //return new PrintCommand("not doing anything in autonomous");
  }


  public void UpdateTrajectoryPIDValues(){
    DriveConstants.kXController.setP(SmartDashboard.getNumber("Drive X P", DriveConstants.kXController.getP()));
    DriveConstants.kYController.setP(SmartDashboard.getNumber("Drive Y P", DriveConstants.kYController.getP()));
    DriveConstants.kXController.setI(SmartDashboard.getNumber("Drive X I", DriveConstants.kXController.getI()));
    DriveConstants.kYController.setI(SmartDashboard.getNumber("Drive Y I", DriveConstants.kYController.getI()));
    DriveConstants.kXController.setD(SmartDashboard.getNumber("Drive X D", DriveConstants.kXController.getD()));
    DriveConstants.kYController.setD(SmartDashboard.getNumber("Drive Y D", DriveConstants.kYController.getD()));
    
    DriveConstants.kThetaController.setP(SmartDashboard.getNumber("Theta P",DriveConstants.kYController.getP()));
    DriveConstants.kThetaController.setI(SmartDashboard.getNumber("Theta I",DriveConstants.kXController.getI()));
    DriveConstants.kThetaController.setD(SmartDashboard.getNumber("Theta D",DriveConstants.kYController.getD()));
  }
}
