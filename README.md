# FRC Robot Project

## Overview
This project is designed for an FRC (FIRST Robotics Competition) robot, which is a property of FEDS. It includes various subsystems such as the elevator and swerve drive, and a safety manager to monitor the health of these subsystems.

## Project Structure
- src/main/java/frc/robot/
    - subsystems/: Contains all the subsystems of the robot.
        - elevator/: Contains the ElevatorSubsystem class.
        - swerve/: Contains the SwerveSubsystem class.
    - utils/: Contains utility classes such as SafetyManager.
    - constants/: Contains constant values used throughout the project.

## Subsystems
### ElevatorSubsystem
The ElevatorSubsystem class controls the elevator mechanism of the robot. It uses a TalonFX motor and a PID controller to maintain the desired position.

### SwerveSubsystem
The SwerveSubsystem class controls the swerve drive mechanism of the robot. It manages the speed and direction of each swerve module.

### SubsystemABS
The SubsystemABS is an abstract base class for all subsystems. It provides common functionality such as network table setup and shuffleboard integration.

## SafetyManager
The SafetyManager class monitors the health of all subsystems. It uses a registry pattern to dynamically create instances of subsystems and check their health status.

## Getting Started
### Prerequisites
- Java Development Kit (JDK) 11 or higher
- Gradle
- WPILib (FRC library)

### Installation
1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/your-repo.git
   ```
2. Navigate to the project directory:
   ```sh
   cd your-repo
   ```
3. Build the project using Gradle:
   ```sh
   ./gradlew build
   ```

### Running the Project
To deploy the code to the robot, use the following command:
```sh
./gradlew deploy
```

### Testing
To run the tests, use the following command:
```sh
./gradlew test
```

## Usage
### ElevatorSubsystem
To set the desired position of the elevator:
```java
ElevatorSubsystem elevator = new ElevatorSubsystem(Subsystems.ELEVATOR, "Elevator");
elevator.setDesiredPosition(100.0);
```

### SafetyManager
To initialize and check the health of subsystems:
```java
SafetyManager.init();
SafetyManager.checkSubsystems();
```

## Contributing
1. Fork the repository.
2. Create a new branch (git checkout -b feature-branch).
3. Make your changes.
4. Commit your changes (git commit -m 'Add some feature').
5. Push to the branch (git push origin feature-branch).
6. Open a pull request.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- WPILib for providing the FRC library.
- FIRST Robotics Competition for the inspiration and framework.