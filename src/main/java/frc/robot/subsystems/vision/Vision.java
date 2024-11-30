package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;
import frc.robot.utils.ObjectType;
import frc.robot.utils.VisionObject;

public class Vision extends SubsystemABS {
    ShuffleboardTab tab;
    Boolean isVisionActive;
    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    NetworkTable limelightTable;
    VisionObject visionObject;

    public Vision(Subsystems part, String tabName) {
        super(part, tabName);
        limelightTable = networkTableInstance.getTable("limelight");
    }

    @Override
    public void init() {
        isVisionActive = true;
        tab = getTab();

        // Initialize preferences with default values
        Preferences.initDouble("LEDMode", 3); // Default to LEDs on
        Preferences.initDouble("TargetThreshold", 10.0); // Example threshold
        commands();
        visionObject = new VisionObject(0, 0, 0, ObjectType.INFINITE_CHARGE_BALLS);
    }

    @Override
    public void periodic() {
        // Dynamically apply preferences
        int ledMode = (int) Preferences.getDouble("Vision/LEDMode", 3);
        double targetThreshold = Preferences.getDouble("Vision/TargetThreshold", 10.0);

        // Apply LED mode
        limelightTable.getEntry("ledMode").setNumber(ledMode);

        visionObject.SetNTEntry(limelightTable);
    }

    @Override
    public void simulationPeriodic() {
        visionObject.update(
                Math.random() * 100 + 1, Math.random() * 100 + 1, Math.random() * 100 + 1
        );
    }

    @Override
    public void setDefaultCommand() {

    }

    @Override
    public boolean isHealthy() {
        return isVisionActive;
    }

    @Override
    public void Failsafe() {

    }

    public double getTargetX() {
        return visionObject.getX();
    }

    public double getTargetY() {
        return visionObject.getY();
    }

    public double getDistance(){
        return visionObject.getDistance();
    }

    public boolean hasTarget(){
        return visionObject.getArea() > 0;
    }

    public void commands() {
        tab.add(getName() + "/Commands/Following Distance", 1);
    }
}
