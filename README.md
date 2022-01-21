# Swerve-Offseason
This is the basic offseason Drivetrain that we use, it works on SDS MK4 speed 2 and further and better instructions will be uploaded soon.

The Program used for generating Autos is seen here, https://github.com/FRCTeam2910/PathViewer. 

##Alignment instructions

*Video can be found in the programing frc channel*

0. Build the Robot connected to school wifi to allow all online features

1. In the `Constans` class:
    2. Set all of the `angleOffset` constants to `angleOffset = 0`.

2. Deploy the code to your robot.
    > NOTE: The robot isn't drivable quite yet, we still have to setup the module offsets

3. Turn the robot on its robotcart and align all the wheels so they are facing in the forwards direction.
    > NOTE: The wheels will be pointed forwards (not backwards) when modules are turned so the large bevel gears are towards the left side of the robot. When aligning the wheels they must be as straight as possible. It is recommended to use a long strait edge such as a piece of 2x1 in order to make the wheels straight.

4. Open Shuffleboard and press the New button to refresh the shuffleboard page

5. Click on the `drivetrain` tab

6. Record the angles of each module using the angle put onto Shuffleboard. The values are named
    `Front Left Module Angle`, `Front Right Module Angle`, etc.

7. Set the values of the `angleOffset` to `angleOffset = <the angle you recorded>`
    > NOTE: All angles must be in degrees.

8. Re-deploy and try to drive the robot forwards. All the wheels should stay parallel to each other. If not go back to
    step 3.

9. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel
    that is spinning in the incorrect direction. i.e `-Math.toRadians(<angle> + 180.0)`.
    
##Auto Instructions

*Better Auto instructions will uploaded soon with a possible video*

0. Go to PathViewer and download it

1. Use Command "  Cnrl `  " to open the VS Code command consle

2. Run Command `./gradlew run` in the consle

3. A map of the field will open up use right click to set points for the path

4. Once the path is created select `Save as` and the path will be downloaded as a file
 
5. Make a file in the resources/autos folder by creating a file and naming it `pathname.path`

6. Paste the path saved in step 4 into the empty file made in step 5
 
7. Go to the Auto Trajectorys folder and initialise it
 
8. Go to the Auto Chooser folder and make it into a command

    Steps 6 and 7 will be expanded apon shortly, i will also most likely make an instructional video
