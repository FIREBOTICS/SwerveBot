# SwerveBot
Swerve chassis code, for use in demos and as a base for season code

[Driver XBox Controller Scheme](https://www.padcrafter.com/?templates=Driver%7COperator&plat=0%7C0&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&yButton=Speed+Up%7C&aButton=Slow+Down%7C&dpadUp=%7C&leftTrigger=Lock%7C&leftStick=X%2FY+Movement%7C&rightStickClick=Rotation%7C&startButton=Toggle+Field+Oriented%7C&backButton=Reset+Heading%7C#)

## New in 2025
- [x] ReduxSwerveModule, CanCoderSwerveModule, and DataPortSwerveModule
- [x] event-deploy and commit recording into DataLog (https://docs.advantagekit.org/getting-started/installation/version-control/#event-deploy) 
- [x] Subsystem logging with Epilogue, custom loggers for third party libraries (https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html)
- [x] isFieldRelative implementation simplified
- [-] SHELVED: Use Sendables for subsystem telemetry -- abandoned because @Logged is easier
