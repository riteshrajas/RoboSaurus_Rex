package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotMap;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class SwerveSubsystem extends SubsystemABS {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final ShuffleboardTab tab;
    private final Pigeon2 pigeonIMU;
    private CommandXboxController driverController;
    private double i = 1.0;

    private String tabName;

    public SwerveSubsystem(Subsystems part, String tabName, int pigeonIMUID, CommandXboxController driverController) {
        super(part, tabName);
        this.tab = Shuffleboard.getTab(tabName);
        this.tabName = tabName;
        this.pigeonIMU = new Pigeon2(pigeonIMUID);  // Initialize Pigeon IMU
        this.driverController = driverController;
    }

    @Override
    public void init() {
        // Initialization logic
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // Simulation-specific periodic tasks
    }

    @Override
    public void setDefaultCommand() {
        drivetrain.setDefaultCommand(new GenericDrivetrain(driverController));
    }

    class GenericDrivetrain extends Command {

        private final CommandXboxController driverController;

        public GenericDrivetrain(CommandXboxController driverController) {
            addRequirements(drivetrain);
            this.driverController = driverController;
            tab.add(tabName + "/LeftX", i);
            tab.add(tabName + "/LeftY", i);
            tab.add(tabName + "/RightX", i);
            tab.add(tabName + "/RightY", i);
        }

        @Override
        public void execute() {
            super.execute();
            i = (Math.random() * 2) - 1; // randomize the value of i from -1 to 1
            new ParallelCommandGroup(
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(-driverController.getLeftY() * RobotMap.SafetyMap.kMaxSpeed * SwerveConstants.MaxSpeed * RobotMap.SafetyMap.kMaxSpeedChange)
                            .withVelocityY(driverController.getLeftX() * RobotMap.SafetyMap.kMaxSpeed * SwerveConstants.MaxSpeed * RobotMap.SafetyMap.kMaxSpeedChange)
                            .withRotationalRate(driverController.getRightX() * SwerveConstants.MaxAngularRate))
            );
        }
    }

    class FODC extends Command {

    }
}