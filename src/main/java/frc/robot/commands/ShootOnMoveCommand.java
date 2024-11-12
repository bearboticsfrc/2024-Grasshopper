package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.bearbotics.util.ParametricPoint;
import frc.robot.subsystems.SwerveSubsystem;
import java.time.Duration;
import java.time.LocalTime;
import java.util.ArrayList;

public class ShootOnMoveCommand extends SequentialCommandGroup {
  LocalTime now;
  private SwerveSubsystem swerve;
  private ArrayList<ParametricPoint> tupleSample = new ArrayList<ParametricPoint>();

  public ShootOnMoveCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  Pose2d poseSupplier() {
    now = LocalTime.now();
    return this.swerve.getSwerveDrive().getPose();
  }

  public void addElement() {
    Pose2d pose = poseSupplier();
    ParametricPoint pOfNow = new ParametricPoint(now, pose.getX(), pose.getY());
    this.tupleSample.add(pOfNow);
  }

  private class DiscretizedTaylorApprox {
    private double[] cofXSeries;
    private double[] cofYseries;

    private double[] xSeries;
    private double[] ySeries;
    private double[] tSeries;
    private ParametricPoint old;

    public DiscretizedTaylorApprox() {
      xSeries = new double[tupleSample.size()];
      ySeries = new double[tupleSample.size()];
      tSeries = new double[tupleSample.size()];

      for (int i = 0; i < tupleSample.size(); i++) {
        ParametricPoint sample = tupleSample.get(i);
        // grab delta
        try {
          old = tupleSample.get(i - 1);
        } catch (IndexOutOfBoundsException e) {
          old = new ParametricPoint(LocalTime.MIDNIGHT, 0, 0);
        }
        double dT = Duration.between(sample.getT(), old.getT()).getSeconds();
        double dX = sample.getX() - old.getX();
        double dY = sample.getY() - old.getY();
        tSeries[i] = dT;
        xSeries[i] = dX;
        ySeries[i] = dY;
      }
    }
  }
}
