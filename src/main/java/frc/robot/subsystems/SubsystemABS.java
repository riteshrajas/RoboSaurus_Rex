package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemABS extends SubsystemBase {
    private Subsystems part;
    private static ShuffleboardTab tab;
    protected static  NetworkTable ntTable;
    protected  static String tabName;

    public SubsystemABS(Subsystems part, String tabName) {
        this.tabName = tabName;
        this.part = part;
        try {
            this.tab = Shuffleboard.getTab(tabName);
        } catch (IllegalArgumentException e) {
            this.tab = Shuffleboard.getTab(tabName + " - New");
        };
        setupNetworkTables(part.toString());
        init();


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

    public void setupNetworkTables(String part) {
        ntTable = NetworkTableInstance.getDefault().getTable(part);
    }


    public abstract void init();
    @Override
    public abstract void periodic();

    @Override
    public abstract void simulationPeriodic();

    public abstract void setDefaultCommand();
}