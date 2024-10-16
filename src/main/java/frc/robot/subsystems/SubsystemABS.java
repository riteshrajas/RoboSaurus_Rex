package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemABS extends SubsystemBase {
    private Subsystems part;
    private ShuffleboardTab tab;

    public SubsystemABS(Subsystems part, String tabName) {
        init();
        this.part = part;
        try {
            this.tab = Shuffleboard.getTab(tabName);
        } catch (IllegalArgumentException e) {
            this.tab = Shuffleboard.getTab(tabName + " - New");
        }
    }

    public Subsystems getPart() {
        return part;
    }

    public void setPart(Subsystems part) {
        this.part = part;
    }

    public ShuffleboardTab getTab() {
        return tab;
    }

    public abstract void init();
    @Override
    public abstract void periodic();

    @Override
    public abstract void simulationPeriodic();

    public abstract void setDefaultCommand();
}