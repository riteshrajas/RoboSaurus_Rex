package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Smooth;


public class FollowerCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    private  ShuffleboardTab tab;
    private Smooth X_filter;
    private Smooth Y_filter;

    private PIDController distancePID;
    private PIDController strafePID;
    private PIDController rotationalPID;

    public FollowerCommand(SwerveSubsystem swerveSubsystem , Vision vision ) {
        this.swerveSubsystem = swerveSubsystem;
        this.vision = vision;
        addRequirements(this.swerveSubsystem , this.vision);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        tab = Shuffleboard.getTab("Command Center");
        X_filter = new Smooth(5);
        Y_filter = new Smooth(2);

        distancePID = new PIDController(0.1,0,0);
        strafePID = new PIDController(0.085,0,0);
        rotationalPID = new PIDController(0.1,0,0);

        distancePID.setSetpoint(1);
        strafePID.setSetpoint(0);
        rotationalPID.setSetpoint(0);

        distancePID.setTolerance((0.1));
        rotationalPID.setTolerance(1);
        strafePID.setTolerance(1);

        rotationalPID.reset();
        strafePID.reset();
        distancePID.reset();

        try {
            tab.add("Distance PID" , distancePID);
            tab.add("Strafe PID" , strafePID);
            tab.add("Rotational PID" , rotationalPID);
        }
        catch (IllegalArgumentException ignored ){
            DriverStation.reportError("Restart ShuffleBoard", false);
        }




    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
     * until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double X_smooth = X_filter.calculate(vision.getTargetX());
            double Y_smooth = Y_filter.calculate(vision.getDistance());

            // Calculating PID Outputs
            double Output_linear = MathUtil.clamp(distancePID.calculate(Y_smooth) , -RobotMap.SafetyMap.kFollowerCommand , RobotMap.SafetyMap.kFollowerCommand);
            double Output_strafe = MathUtil.clamp(strafePID.calculate(X_smooth) , -RobotMap.SafetyMap.kFollowerCommand , RobotMap.SafetyMap.kFollowerCommand);
            double Output_rotation = MathUtil.clamp(rotationalPID.calculate(vision.getDistance()) , -RobotMap.SafetyMap.kFollowerCommand , RobotMap.SafetyMap.kFollowerCommand);

            // Acting Upon the Output
            swerveSubsystem.drivetrain.setControl(swerveSubsystem.drive
                    .withVelocityX(Output_linear)
                    .withVelocityY(Output_strafe)
                    .withRotationalRate(Output_rotation)

            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
}
