package org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util.oldCode.ComputedSpline;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util.oldCode.Spline;

public final class SplineHelper {
    public ParametricSpline computeSpline(double[] x, double[] y)
    {
        int n = x.length;
        double[] distances = new double[n];
        distances[0] = 0;
        double totalDist = 0;

        for (int i = 1; i < n; i++)
        {
            double dx = x[i] - x[i - 1];
            double dy = y[i] - y[i - 1];
            double dist = Math.sqrt(dx * dx + dy * dy);
            totalDist += dist;
            distances[i] = totalDist;
        }

        PolynomialSplineFunction f0 = new SplineInterpolator().interpolate(distances, x);
        PolynomialSplineFunction f1 = new SplineInterpolator().interpolate(distances, y);

        //We want a spline that has a more accurately calculated distance.
        double[] xs = new double[(int) totalDist + 1];
        double[] ys = new double[(int) totalDist + 1];

        for (int i = 0; i < totalDist; i++)
        {
            xs[i] = f0.value(i);
            ys[i] = f1.value(i);
        }

        n = xs.length;
        distances = new double[n];
        distances[0] = 0;
        totalDist = 0;

        for (int i = 1; i < n; i ++)
        {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            double dist = Math.sqrt(dx * dx + dy * dy);
            totalDist += dist;
            distances[i] = totalDist;
        }

        f0 = new SplineInterpolator().interpolate(distances, xs);
        f1 = new SplineInterpolator().interpolate(distances, ys);


        return new ParametricSpline(f0, f1, totalDist, distances);
    }

    @Deprecated
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
        spline.xSpline = xSpline;

        Spline ySpline = new Spline(distances, y);
        spline.ys = ySpline.mapSpline(times);
        spline.ySpline = ySpline;

        spline.dist = totalDist;

        return spline;
    }
}
