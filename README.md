## Installation and Getting Started

1. Follow the instructions at https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html and install WPILib  
2. Open `Visual Studio Code.app` found in `~/wpilib/YYYY/vscode/` where `YYYY` is the year number  
3. 

## Contributing Code

`$ git clone REPO_URL`

### Top-level directory layout of a robot project

    .
    ├── src                     # Java source files 
    │   └── main
    │       ├── java
    │       │   ├── robot       # This is where all of your robot code will go
    │       │   │   └── ...
    │       │   └── lib         # The swerve library code lives here, you shouldn't need to change anything in here
    │       │       └── ...
    │       └── deploy          # Files that will be deployed onto the RoboRIO, usually resource files that are needed on the robot
    │           └── ... 
    ├── vendordeps              # Dependency files 
    │   ├── REVLib.json         # Example: REVLib dependency json file
    │   └── ...                 
    ├── build                   # Compiled files. Never change this, except after a corrupted builds, delete it entirely and rebuild
    └── ...                     # Most of the other files are auto-generated, do not change them

### You will primarily be working in `/src/main/java/robot/` 

Main.java
Robot.java
RobotContainer.java
Constants.java
... # ignore other files



## Resources
* Quickstart: https://docs.wpilib.org/en/stable/docs/yearly-overview/returning-quickstart.html
* 