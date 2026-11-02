package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Eight Shaped", group = "Autonomous")
public class AutoEightShaped extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startpos);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    //points here \/
    private final Pose startpos = poseFactory.of(89, 7, 90);
    private final Pose point1 = poseFactory.of(89, 103, 90);
    private final Pose point2 = poseFactory.of(53, 103, 90);
    private final Pose point3 = poseFactory.of(53, 10, 90);
    private final Pose point4 = poseFactory.of(10, 10, 90);
    private final Pose point5 = poseFactory.of(10, 103, 90);
    private final Pose point6 = poseFactory.of(53, 103, 90);
    private final Pose point7 = poseFactory.of(53, 10, 270);
    private final Pose endpos = poseFactory.of(89, 7, 270);

    //paths here \/
    public Path path1() {
        return Paths.line(startpos, point1).linear(startpos, point1);
    }
    public Path path2() {
        return Paths.line(point1, point2).linear(point1, point2);
    }
    public Path path3() {
        return Paths.line(point2, point3).linear(point2, point3);
    }
    public Path path4() {
        return Paths.line(point3, point4).linear(point3, point4);
    }
    public Path path5() {
        return Paths.line(point4, point5).linear(point4, point5);
    }
    public Path path6() { return Paths.line(point5, point6).linear(point5, point6); }
    public Path path7() {
        return Paths.line(point6, point7).linear(point6, point7);
    }
    public Path path8() {return Paths.line(point7, endpos).linear(point7, endpos);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()));
    }
}