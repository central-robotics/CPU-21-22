package org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util;

public final class SplineHelper {
    public final ComputedSpline calculateSpline(float[] x, float[] y, int outputPoints) throws Exception {
        int n = x.length;
        float[] distances = new float[n];
        distances[0] = 0;
        float totalDist = 0;

        for (int i = 1; i < n; i++)
        {
            float dx = x[i] - x[i - 1];
            float dy = y[i] - y[i - 1];
            float dist = (float)Math.sqrt(dx * dx + dy * dy);
            totalDist += dist;
            distances[i] = totalDist;
        }

        float dt = totalDist / (outputPoints - 1);
        float[] times = new float[outputPoints];
        times[0] = 0;

        for (int i = 1; i < outputPoints; i++)
        {
            times[i] = times[i - 1] + dt;
        }
        ComputedSpline spline = new ComputedSpline();

        Spline xSpline = new Spline(distances, x);
        spline.xs = xSpline.mapSpline(times);
        Spline ySpline = new Spline(distances, y);
        spline.ys = ySpline.mapSpline(times);

        return spline;
    }
}
