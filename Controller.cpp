double desiredModelCrankPosition(double t) {
    double whole, frac;
    frac = modf(t, &whole);
    return frac * 360.0;
}

double desiredModelCrankVelocity(double t) {
    return 2 * Pi;
}

class PedalController : public Controller {

public:
    PedalController(double aKp, double aKv) : Controller(), kp(aKp), kv(aKv) {}

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override {
        double t = s.getTime();
        double crankDes = desiredModelCrankPosition(t);
        double crankVelDes = desiredModelCrankVelocity(t);

        const Coordinate& crankCoord = _model->getCoordinateSet().get("crankAngle");
        double crankPos = crankCoord.getValue(s);
        double crankVel = crankCoord.getSpeedValue(s);

        double pErr = kp * (crankDes - crankPos);
        double vErr = kv * (crankVelDes - crankVel);

        double desAcc = pErr + vErr;

    }
private:
    double kp;
    double kv;

};