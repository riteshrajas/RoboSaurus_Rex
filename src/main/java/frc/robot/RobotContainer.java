// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;



public class RobotContainer
{

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(RobotMap.UsbMap.DRIVER_CONTROLLER);


    public RobotContainer()
    {
        // Configure the trigger bindings
        SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Subsystems.SWERVE_DRIVE, Subsystems.SWERVE_DRIVE.getNetworkTable(), RobotMap.SensorMap.GYRO_PORT, driverController);
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(Subsystems.ELEVATOR, Subsystems.ELEVATOR.getNetworkTable());
        configureBindings();
        swerveSubsystem.setDefaultCommand();
    }



    private void configureBindings()
    {

    }


    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return null;
    }

}
