package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemABS {

    private TalonFX elevatorMotor;
    private PIDController pidController;
    private DoubleSupplier positionSupplier;
    private double desiredPosition;
    private ShuffleboardTab tab;

    public ElevatorSubsystem(Subsystems part, String tabName) {
        super(part, tabName);
    }


    @Override
    public void init() {
        tab = getTab();
        desiredPosition = 0;
        elevatorMotor = new TalonFX(RobotMap.ElevatorMap.ELEVATOR_MOTOR);
        pidController = new PIDController(0.1, 0.0, 0.0); // Initialize PID controller with example gains
        positionSupplier = ntTable.getDoubleTopic("elevator_position").getEntry(0);
        printToShuffleboard();
    }

    @Override
    public void periodic() {
        // Calculate the output using the PID controller
        double currentPosition = getCurrentPosition().getAsDouble();
        double output = pidController.calculate(currentPosition, desiredPosition);
        setElevatorVoltage(output);

    }

    private void printToShuffleboard() {
        tab.addNumber(tabName+"/Elevator Position", () -> positionSupplier.getAsDouble());
        tab.addNumber(tabName+"/Desired Position", () -> desiredPosition);
        tab.addNumber(tabName+"/Elevator Voltage", () -> elevatorMotor.getMotorVoltage().getValueAsDouble());
    }

    public void setElevatorVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
        elevatorMotor.feed();
    }

    public void setDesiredPosition(double position) {
        this.desiredPosition = position;
    }

    private DoubleSupplier getCurrentPosition() {
        // Return the current position of the elevator using the encoder
        return positionSupplier;
    }

    @Override
    public void simulationPeriodic() {
        // Simulation-specific periodic tasks
    }

    @Override
    public void setDefaultCommand() {
        // Set the default command for the subsystem here
    }

    @Override
    public boolean isHealthy() {
        // Check if the elevator subsystem is healthy
        return true;
    }

    @Override
    public void Failsafe() {

    }
}