package org.firstinspires.ftc.teamcode.teleop.systems.utils;

public class CreateSlothComments {
    /*
You do one normal install, and after that you push code changes with a single Gradle task that takes about a second instead of a full install.

One-time setup

1. Full install. Connect to the robot’s Wi-Fi and run the normal TeamCode configuration in Android Studio. This puts the Sloth runtime on the robot. Repeat this step whenever the setup itself changes (see “When you need a full install” below).

2. Create a deploySloth run configuration:

At the top of Android Studio, open the run-configuration dropdown (it says TeamCode) and choose Edit Configurations…
Click + and choose Gradle.
Name it deploySloth.
In Gradle project, type :TeamCode. Type it yourself rather than copying from the README.
In Tasks (also called Run), type deploySloth. It won’t autocomplete, but it works.
Click OK.

3. Make normal installs clear the old Sloth code:

Open Edit Configurations… again and select your TeamCode configuration.
At the bottom, under Before launch, click + and choose Run Gradle task.
Set the project to :TeamCode and the task to removeSlothRemote.
Move it to the top of the list so it runs before the build, then click OK.

This step matters. Without it, hot-loaded code saved on the robot keeps overriding your fresh full installs.

Everyday workflow
Connect to the robot’s Wi-Fi.
Edit your code.
Select deploySloth in the dropdown and click Run.
The new code takes effect when the current OpMode ends, so it’s safe to deploy while something is running. Then select and run your OpMode again.

From Terminal, ./gradlew :TeamCode:deploySloth should do the same thing. Your Mac has to be connected to the robot over ADB for that, which Android Studio normally handles. If it says no device is connected, run adb connect 192.168.43.1:5555 first.

What Sloth can and can’t hot-reload

deploySloth works for: any Java changes under org.firstinspires.ftc.teamcode, which is all of your OpModes, subsystems, Constants, and tuners.

You need a full install (the TeamCode configuration) when you:

Change anything in a Gradle file, such as adding a library or changing a version.
Change the FtcRobotController module or an AndroidManifest.xml.
Add or remove @Pinned on a class.
See behavior that doesn’t match your code. When in doubt, do a full install.
Things that will trip you up
Static variables reset on every reload. startPose, isRed, inAuto, the transportState static fields, and tuned @Configurable values all go back to their defaults. At a competition, don’t run deploySloth between auto and teleop, or teleop loses the pose that auto saved.
Changes apply when an OpMode ends. If something seems unchanged, stop the OpMode and start it again.
Old code keeps coming back. If a full install doesn’t seem to take effect, leftover Sloth code on the robot is overriding it. Clear it by hand:
bash
adb shell rm -rf /storage/emulated/0/FIRST/dairy/sloth/*

Then do a full install again.

Sloth’s dashboard runs at http://192.168.43.1:8001 and Panels at its usual address. Both update @Configurable and @Config values live, which is how you’ll tune the flywheel P and F without redeploying.

Source: Sloth README
     */
}
