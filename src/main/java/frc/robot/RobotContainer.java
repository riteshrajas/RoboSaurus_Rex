// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GamePadViberatorCommand;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;


public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(RobotMap.UsbMap.DRIVER_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(RobotMap.UsbMap.OPERATOR_CONTROLLER);

    private final SwerveSubsystem swerveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final LauncherSubsystem launcherSubsystem;
    private final Vision visionSubsystem;


    public RobotContainer() {
        // Configure the trigger bindings
        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE ,
                Subsystems.SWERVE_DRIVE.getNetworkTable() ,
                RobotMap.SensorMap.GYRO_PORT ,
                driverController
        );
        elevatorSubsystem = new ElevatorSubsystem(
                Subsystems.ELEVATOR ,
                Subsystems.ELEVATOR.getNetworkTable()
        );
        launcherSubsystem = new LauncherSubsystem(
                Subsystems.TURRET ,
                Subsystems.TURRET.getNetworkTable()
        );
        visionSubsystem = new Vision(
                Subsystems.VISION ,
                Subsystems.VISION.getNetworkTable()
        );



        configureBindings();
        swerveSubsystem.setDefaultCommand(); /*       swerveSubsystem.setBetaDefaultCommand();    */

    }


    private void configureBindings() { }


    public Command getAutonomousCommand() {return null; }


//     DO NOT REMOVE
    public SubsystemABS[] SafeGuardSystems() {
        return new SubsystemABS[] {
                swerveSubsystem ,
                elevatorSubsystem ,
                launcherSubsystem ,
                visionSubsystem
        };
    }
    public Object[] TestCommands() {
        return new Object[] {
                "Driver Controller Vibration" , new GamePadViberatorCommand(driverController.getHID()) ,
                "Operator Controller Vibration" , new GamePadViberatorCommand(operatorController.getHID())
        };
    }
}
