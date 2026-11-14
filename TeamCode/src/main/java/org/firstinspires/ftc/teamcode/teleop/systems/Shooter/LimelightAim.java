package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;

import java.util.List;

/**
 * Limelight aiming override. When odometry says the robot is inside the Limelight zone for the current side
 * (GoalTargets.leftSide) and the Limelight sees that side's AprilTag, the turret angle and the LUT distance come
 * from the Limelight instead of odometry. Anywhere else, or with no fresh tag, the Shooter aims from odometry as before.
 * The Shooter runs this every updateAim(), so every opmode that uses the Shooter gets it.
 *
 * The Limelight is on the turret, so it turns with it. From one camera frame, the tag position in the turret frame:
 *   usePointOfInterest (default): the pipeline's point of interest (POI) tracking puts tx / ty on the goal point
 *     instead of the tag center. Bearing = cameraYawOffsetDeg - tx, distance from ty with the camera height and tilt:
 *     forward distance = (poiHeightIn - cameraHeightIn) / tan(cameraPitchDeg + ty). The POI is the goal point, so the
 *     tag field positions aren't used. No 3D pose needed from the Limelight.
 *   useRobotSpacePose: the Limelight's robot-space tag pose, with the camera pose (tilt included) set in
 *     the Limelight web UI relative to the turret's rotation center. Floor part only.
 *   otherwise (level camera): bearing = cameraYawOffsetDeg - tx (tx + = tag right of the crosshair), range = 3D tag
 *     pose in camera space, floor part, plus the camera's offset on the turret.
 *   Either way the range needs the right tag size in the Limelight pipeline.
 * The tag and the goal point aren't always the same spot, so the goal point is found from the tag plus the
 * (goal point - tag) offset in field inches, turned into the turret frame with the heading. Only that short offset
 * uses the heading, so heading drift barely matters. With the tag position set to the goal point the offset is 0
 * and the heading isn't used at all.
 *   turret angle = turret angle when the frame was taken (encoder, or the command without one) + bearing to the goal point
 *                  - how much the robot turned since that frame
 *   LUT distance = turret center -> goal point (same distance the LUTs were tuned with on odometry) + distanceOffsetIn
 * The frame is a few tens of ms old, so the turret angle and heading at that moment are looked up in a short history.
 *
 * To set up (POI mode): 4 AprilTag pipelines (red right/left, blue right/left), each with the right tag size, an ID
 * filter for that side's tag (tx / ty come from the primary target, so only that tag should be in it) and the POI offset
 * from the tag to the goal point. Here: camera height / pitch, POI height, camera position on the turret, zones. Then stand in a zone right after relocalizing and compare the "Limelight"
 * and "odometry" telemetry: they should agree within a degree or two and an inch or two.
 *
 * Frames: field inches, heading degrees counterclockwise (same as FollowerPose). Turret angles: degrees,
 * 0 = robot forward, counterclockwise positive (same as TurretPollen).
 */
@Configurable
public class LimelightAim {
    // -------------- On / off --------------
    public static boolean enabled = true;
    public static boolean enabledInAuto = true; // also needs enabled
    public static boolean useForTurret = true;
    public static boolean useForLut = true;
    // Shoot on the move still applies on top: its aim shift (deg) and distance shift (in) are added
    public static boolean addShootOnTheMove = true;

    // -------------- Hardware --------------
    public static String hardwareName = "limelight";
    // Pipeline per alliance and side (set up 4 AprilTag pipelines in the Limelight web UI). Switched automatically
    // when the alliance or side (GoalTargets.leftSide) changes.
    public static int redRightPipeline = 0;
    public static int redLeftPipeline = 1;
    public static int blueRightPipeline = 2;
    public static int blueLeftPipeline = 3;
    public static int pollRateHz = 100;

    // -------------- Tags --------------
    // Tag ID per alliance and side. -1 = accept any tag (uses the biggest one in view).
    public static int redRightTagId = -1;
    public static int redLeftTagId = -1;
    public static int blueRightTagId = -1;
    public static int blueLeftTagId = -1;
    // Where each tag is on the field (inches, red; blue is mirrored x -> fieldWidth - x, like GoalTargets).
    // PLACEHOLDERS: set to the GoalTargets points, which aims straight at the tag. Measure the real tag spots.
    public static double redRightTagX = 58;
    public static double redRightTagY = 58;
    public static double redLeftTagX = 62;
    public static double redLeftTagY = 83;

    // -------------- Camera on the turret --------------
    // true: aim at the pipeline's point of interest (set the POI offset from the tag to the goal point in each
    // pipeline). Uses cameraHeightIn / cameraPitchDeg / poiHeightIn below, plus cameraForwardIn / cameraLeftIn /
    // cameraYawOffsetDeg. useRobotSpacePose and the tag field positions are then unused.
    public static boolean usePointOfInterest = true;
    // PLACEHOLDERS: measure them. Heights from the floor (inches); pitch = camera tilt, + = tilted up (deg).
    public static double cameraHeightIn = 12;
    public static double cameraPitchDeg = 20;
    public static double poiHeightIn = 40; // height of the point of interest (the goal point)

    // usePointOfInterest = false only:
    // true: the tag position comes from the Limelight's robot-space pose. Enter the camera pose in the Limelight web
    // UI (camera pose in robot space) as measured from the turret's rotation center, with "robot forward" = where the
    // turret points: LL Forward / LL Right / LL Up in meters, plus roll / pitch (the tilt) / yaw. Handles a tilted
    // camera. The three camera settings below are then unused.
    // false: level camera only (no tilt); the camera position and yaw come from the three settings below.
    public static boolean useRobotSpacePose = true;
    // Limelight docs say robot space +y = right. If a tag to the turret's LEFT gives a negative "tag left" in
    // telemetry, flip this.
    public static boolean flipRobotSpaceSide = false;
    // Point of interest and level camera modes: lens position from the turret's rotation center (inches, turret frame:
    // forward = where the turret points, left = 90 deg counterclockwise of that), and which way it points
    // (deg, + = turned left of the turret).
    public static double cameraForwardIn = 0;
    public static double cameraLeftIn = 0;
    public static double cameraYawOffsetDeg = 0;

    // -------------- Zones (red, robot center from odometry; blue mirrored) --------------
    // The Limelight is only used while the robot is inside the zone for the side it's aiming at.
    // PLACEHOLDERS: set them to where the camera can actually see that side's tag.
    public static double rightZoneMinX = 20;
    public static double rightZoneMaxX = 70;
    public static double rightZoneMinY = 0;
    public static double rightZoneMaxY = 50;
    public static double leftZoneMinX = 20;
    public static double leftZoneMaxX = 70;
    public static double leftZoneMinY = 90;
    public static double leftZoneMaxY = 141.5;

    // -------------- Timing and checks --------------
    public static double maxFrameAgeMs = 120;      // older frames (tag lost) -> back to odometry
    // Turret angle when the frame was taken: from the turret encoder if it has a reading (TurretPollen), otherwise the
    // command. The servo reaches a commanded angle a bit later, so then it's the command from servoLagMs before.
    public static boolean useTurretEncoder = true;
    public static double servoLagMs = 30;
    // Ignore the Limelight if it disagrees with odometry by more than this (wrong tag, glare). 0 = never ignore.
    public static double maxDisagreementDeg = 25;
    public static double distanceOffsetIn = 0;     // trim on the Limelight distance (if its range reads short/long)

    // -------------- Telemetry (outputs) --------------
    @IgnoreConfigurable public static boolean connected;     // false = no Limelight with hardwareName
    @IgnoreConfigurable public static boolean inZone;        // robot is in the zone for the current side
    @IgnoreConfigurable public static boolean tagSeen;       // the right tag is in a fresh frame
    @IgnoreConfigurable public static boolean active;        // the override is being used this loop
    @IgnoreConfigurable public static int seenTagId = -1;
    @IgnoreConfigurable public static int currentPipeline = -1; // last pipeline switched to
    @IgnoreConfigurable public static double frameAgeMs;
    @IgnoreConfigurable public static double tx, ty, tagRangeIn; // with POI: tx / ty / range are to the POI
    @IgnoreConfigurable public static double tagForwardIn, tagLeftIn; // tag (or POI) in the turret frame (inches)
    @IgnoreConfigurable public static double limelightTurretAngle, limelightDistance; // before shoot on the move
    @IgnoreConfigurable public static double odometryTurretAngle, odometryDistance;   // same, from odometry
    @IgnoreConfigurable public static String turretAngleSource = ""; // "encoder" or "command"
    @IgnoreConfigurable public static String status = "";

    // -------------- Result of the last update() --------------
    public double turretAngle;  // degrees, robot frame (no driver offset, no shoot on the move)
    public double distance;     // inches, turret center -> goal point

    private final Limelight3A limelight;

    // Short history of (time, commanded turret angle, measured turret angle, heading), to look up where the turret
    // pointed when a frame was taken
    private static final int HISTORY = 128;
    private final long[] historyNanos = new long[HISTORY];
    private final double[] historyTurret = new double[HISTORY];
    private final double[] historyMeasured = new double[HISTORY]; // NaN = no encoder reading
    private final double[] historyHeading = new double[HISTORY];
    private int historyCount = 0, historyNext = 0;

    public LimelightAim(HardwareMap hardwareMap) {
        // tryGet: opmodes still run on a robot config without a Limelight (the override just stays off)
        limelight = hardwareMap.tryGet(Limelight3A.class, hardwareName);
        connected = limelight != null;
        currentPipeline = -1;
        if (limelight != null) {
            limelight.setPollRateHz(pollRateHz);
            switchPipelineIfNeeded();
            limelight.start();
        }
        active = inZone = tagSeen = false;
        status = connected ? "" : "no device named " + hardwareName;
    }

    /**
     * Call once per loop, before aiming. odometryAngle / odometryDist = what odometry aiming would use this loop
     * (for the disagreement check and telemetry). Returns true if the Shooter should use turretAngle / distance.
     */
    public boolean update(double goalX, double goalY, double odometryAngle, double odometryDist) {
        odometryTurretAngle = odometryAngle;
        odometryDistance = odometryDist;
        active = false;
        tagSeen = false;
        inZone = inZone(FollowerPose.botX, FollowerPose.botY);

        if (limelight == null) return false;
        // Before the zone check, so the camera is already on the right pipeline when the robot drives into a zone
        switchPipelineIfNeeded();
        if (!enabled || (inAuto && !enabledInAuto)) { status = "off"; return false; }
        if (!inZone) { status = "out of zone"; return false; }

        // ----- Fresh frame with the right tag -----
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { status = "no target"; return false; }
        // Right after a switch the latest frame can still be from the old pipeline
        if (result.getPipelineIndex() != currentPipeline) { status = "switching pipeline"; return false; }
        frameAgeMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        if (frameAgeMs > maxFrameAgeMs) { status = "frame too old"; return false; }

        LLResultTypes.FiducialResult tag = pickTag(result.getFiducialResults(), wantedTagId());
        if (tag == null) { status = "wrong / no tag"; return false; }
        tx = tag.getTargetXDegrees();

        // ----- Tag in the turret frame (when the frame was taken), inches -----
        double tagForward, tagLeft;
        if (usePointOfInterest) {
            // With POI tracking on, the result's tx / ty point at the POI (the goal point), not the tag center
            tx = result.getTx();
            ty = result.getTy();
            double angleUp = Math.toRadians(cameraPitchDeg + ty);
            double forwardDist = (poiHeightIn - cameraHeightIn) / Math.tan(angleUp); // lens -> POI, along the camera's aim
            if (!(forwardDist > 0) || Double.isInfinite(forwardDist)) { status = "bad ty (check heights / pitch)"; return false; }
            tagRangeIn = forwardDist / Math.cos(Math.toRadians(tx)); // floor distance along the bearing
            double bearing = Math.toRadians(cameraYawOffsetDeg - tx);
            tagForward = cameraForwardIn + tagRangeIn * Math.cos(bearing);
            tagLeft = cameraLeftIn + tagRangeIn * Math.sin(bearing);
        } else if (useRobotSpacePose) {
            // The Limelight's "robot" is the turret: it already turned the tag into the turret frame using the camera
            // pose from its web UI (tilt included). Robot space: x forward, y RIGHT, z up, origin on the floor.
            Pose3D tagPose = tag.getTargetPoseRobotSpace();
            if (tagPose == null) { status = "no 3D pose"; return false; }
            Position p = tagPose.getPosition().toUnit(DistanceUnit.INCH);
            tagForward = p.x;
            tagLeft = flipRobotSpaceSide ? p.y : -p.y;
            tagRangeIn = Math.hypot(tagForward, tagLeft); // turret center -> tag, floor distance
        } else {
            // Level camera: camera space x right, y down, z out of the lens; keep the floor part
            Pose3D tagPose = tag.getTargetPoseCameraSpace();
            if (tagPose == null) { status = "no 3D pose"; return false; }
            Position p = tagPose.getPosition().toUnit(DistanceUnit.INCH);
            tagRangeIn = Math.hypot(p.x, p.z); // camera -> tag, floor distance
            double bearing = Math.toRadians(cameraYawOffsetDeg - tx);
            tagForward = cameraForwardIn + tagRangeIn * Math.cos(bearing);
            tagLeft = cameraLeftIn + tagRangeIn * Math.sin(bearing);
        }
        tagForwardIn = tagForward;
        tagLeftIn = tagLeft;
        if (!(tagRangeIn > 1)) { status = "bad 3D pose"; return false; }
        seenTagId = tag.getFiducialId();
        tagSeen = true;

        // ----- Turret angle and heading when the frame was taken -----
        long now = System.nanoTime();
        long frameNanos = now - (long) (frameAgeMs * 1e6);
        int frameIndex = historyIndexAt(frameNanos);
        if (frameIndex < 0) { status = "no history yet"; return false; }
        double headingAtFrame = historyHeading[frameIndex];
        double turretAtFrame;
        if (useTurretEncoder && !Double.isNaN(historyMeasured[frameIndex])) {
            turretAtFrame = historyMeasured[frameIndex]; // where the turret really was
            turretAngleSource = "encoder";
        } else {
            turretAtFrame = historyTurret[historyIndexAt(frameNanos - (long) (servoLagMs * 1e6))];
            turretAngleSource = "command";
        }

        // ----- Goal point = tag + (goal point - tag), the offset turned from field into the turret frame -----
        // (with POI the point already is the goal point, so the offset is 0)
        double dx = usePointOfInterest ? 0 : goalX - tagX();
        double dy = usePointOfInterest ? 0 : goalY - tagY();
        double turretField = Math.toRadians(headingAtFrame + turretAtFrame);
        double goalForward = tagForward + dx * Math.cos(turretField) + dy * Math.sin(turretField);
        double goalLeft = tagLeft - dx * Math.sin(turretField) + dy * Math.cos(turretField);

        double turnedSinceFrame = wrap180(FollowerPose.botHeading - headingAtFrame);
        turretAngle = turretAtFrame + Math.toDegrees(Math.atan2(goalLeft, goalForward)) - turnedSinceFrame;
        distance = Math.hypot(goalForward, goalLeft) + distanceOffsetIn;
        limelightTurretAngle = turretAngle;
        limelightDistance = distance;

        if (maxDisagreementDeg > 0 && Math.abs(wrap180(turretAngle - odometryAngle)) > maxDisagreementDeg) {
            status = "disagrees with odometry";
            return false;
        }
        status = "LIMELIGHT";
        active = true;
        return true;
    }

    /**
     * Call once per loop after the turret is commanded, so later frames can look up where it pointed.
     * measuredTurretAngle = turret encoder angle, NaN if there's none.
     */
    public void record(double commandedTurretAngle, double measuredTurretAngle, double headingDeg) {
        historyNanos[historyNext] = System.nanoTime();
        historyTurret[historyNext] = commandedTurretAngle;
        historyMeasured[historyNext] = measuredTurretAngle;
        historyHeading[historyNext] = headingDeg;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    // -------------------------------- HELPERS ---------------------------------

    /** Newest history entry at or before t (the oldest one if t is older than the whole history), -1 if empty. */
    private int historyIndexAt(long t) {
        if (historyCount == 0) return -1;
        int index = -1;
        for (int i = 1; i <= historyCount; i++) {
            int k = (historyNext - i + HISTORY) % HISTORY; // newest first
            index = k;
            if (historyNanos[k] <= t) break;
        }
        return index;
    }

    /** The tag with this ID, or the biggest tag in view for -1. */
    private static LLResultTypes.FiducialResult pickTag(List<LLResultTypes.FiducialResult> tags, int id) {
        if (tags == null) return null;
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            if (id >= 0 && t.getFiducialId() != id) continue;
            if (best == null || t.getTargetArea() > best.getTargetArea()) best = t;
        }
        return best;
    }

    /** Pipeline for the current alliance and side. */
    private static int wantedPipeline() {
        if (isRed) return GoalTargets.leftSide ? redLeftPipeline : redRightPipeline;
        return GoalTargets.leftSide ? blueLeftPipeline : blueRightPipeline;
    }

    /** Switches the Limelight pipeline only when it changes (each switch is a request over USB). */
    private void switchPipelineIfNeeded() {
        int wanted = wantedPipeline();
        if (wanted == currentPipeline) return;
        if (limelight.pipelineSwitch(wanted)) currentPipeline = wanted; // on failure, retried next loop
    }

    private static int wantedTagId() {
        if (isRed) return GoalTargets.leftSide ? redLeftTagId : redRightTagId;
        return GoalTargets.leftSide ? blueLeftTagId : blueRightTagId;
    }

    private static double tagX() {
        double redX = GoalTargets.leftSide ? redLeftTagX : redRightTagX;
        return isRed ? redX : GoalTargets.fieldWidth - redX;
    }

    private static double tagY() {
        return GoalTargets.leftSide ? redLeftTagY : redRightTagY;
    }

    /** Is the robot (odometry) inside the zone for the current side? Blue mirrors x like GoalTargets. */
    public static boolean inZone(double x, double y) {
        double minX = GoalTargets.leftSide ? leftZoneMinX : rightZoneMinX;
        double maxX = GoalTargets.leftSide ? leftZoneMaxX : rightZoneMaxX;
        double minY = GoalTargets.leftSide ? leftZoneMinY : rightZoneMinY;
        double maxY = GoalTargets.leftSide ? leftZoneMaxY : rightZoneMaxY;
        if (!isRed) {
            double mirroredMin = GoalTargets.fieldWidth - maxX;
            maxX = GoalTargets.fieldWidth - minX;
            minX = mirroredMin;
        }
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    private static double wrap180(double degrees) {
        return ((degrees % 360) + 540) % 360 - 180;
    }
}
