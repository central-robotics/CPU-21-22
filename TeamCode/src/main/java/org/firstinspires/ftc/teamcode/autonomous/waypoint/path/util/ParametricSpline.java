package org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.Point;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class ParametricSpline {
    public PolynomialSplineFunction xSpline;
    public PolynomialSplineFunction ySpline;
    private final PolynomialSplineFunction xPrimet;
    private final PolynomialSplineFunction yPrimet;
    public double splineDistance;

    public ParametricSpline(PolynomialSplineFunction f0, PolynomialSplineFunction f1, double dist)
    {

        xSpline = f0;
        ySpline = f1;
        xPrimet = xSpline.polynomialSplineDerivative();
        yPrimet = ySpline.polynomialSplineDerivative();
        splineDistance = dist;
    }

    public double getDerivative(double t)
    {
        return yPrimet.value(t) / xPrimet.value(t);
    }

    public double getCurvature(double t)
    {
        return (Math.abs((xPrimet.value(t)*yPrimet.polynomialSplineDerivative().value(t))
                - (yPrimet.value(t) * xPrimet.derivative().value(t))))/(Math.pow(Math.pow(xPrimet.value(t), 2) + Math.pow(yPrimet.value(t), 2), 3/2));
    }
}
