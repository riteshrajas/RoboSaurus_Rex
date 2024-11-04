package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;
import java.util.Map;

public class LauncherSubsystem extends SubsystemABS {
    private Solenoid launcherSolenoidForward;
    private Solenoid launcherSolenoidReverse;
    private TalonFX turretMotor;
    private ShuffleboardTab tab;


    public LauncherSubsystem(Subsystems part, String tabName) {
        super(part, tabName);
    }


    @Override
    public Command getDefaultCommand() {
        return super.getDefaultCommand();
    }

    @Override
    public void init() {
        tab = getTab();
        turretMotor = new TalonFX(RobotMap.LauncherMap.TURRET_MOTOR);

        // Initialize separate solenoids for forward and reverse
        launcherSolenoidForward = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.LauncherMap.LAUNCHER_SOLENOID_FORWARD);
        launcherSolenoidReverse = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.LauncherMap.LAUNCHER_SOLENOID_REVERSE);

        tab.add("Turret Motor", turretMotor);
        tab.addBoolean("Solenoid State", this::getSolenoidState);
        tab.add("Piston Position", this::getPistonPosition)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0, "max", 1));
    }


    private void getPistonPosition(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Position", this::getPistonPosition,null);
    }

    private double getPistonPosition()    {
        return launcherSolenoidForward.get() ? 1.0 : 0.0;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void setDefaultCommand() {
    }

    @Override
    public boolean isHealthy() {
        return true;
    }

    @Override
    public void Failsafe() {

    }

    public Command seekTarget() {
        return null;
    }

    public Command fire() {
        return null;
    }

    public Command stop() {
        return null;
    }

    public Command retract() {
        return null;
    }

    public void setSolenoidState(boolean state) {
        if (state) {
            launcherSolenoidForward.set(true);
            launcherSolenoidReverse.set(false);
        } else {
            launcherSolenoidForward.set(false);
            launcherSolenoidReverse.set(true);
        }
    }

    private boolean getSolenoidState() {
        return launcherSolenoidForward.get();
    }

}