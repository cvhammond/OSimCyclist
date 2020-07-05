#include "Bicycle.h"

void Cyclist::buildBB()
{
    Ground& ground = model.updGround();

    ground.attachGeometry(new Mesh("Geometry/bottombracket.stl"));
}

void Cyclist::buildCrankset()
{
    double xx = 0.00980, yy = 0.00547, zz = 0.00829, xy = 0, xz = 0, yz = 0.00254;

    double cranksetMass = 1.15;
    Vec3 cranksetMassCenter(0.0, 0.0, 0.0199);
    Inertia cranksetInertia = Inertia(xx, yy, zz, xy, xz, yz);
    OpenSim::Body* crankset = new OpenSim::Body("crankset", cranksetMass,
        cranksetMassCenter, cranksetInertia);

    crankset->attachGeometry(new Mesh("Geometry/crankset.stl"));

    Vec3 locationInParent(0), orientationInParent(0, Pi, 0);
    Vec3 locationInBody(0), orientationInBody(0, Pi, 0);
    Ground& ground = model.updGround();

    PinJoint* crankAngle = new PinJoint("crankset", ground,
        locationInParent, orientationInParent,
        *crankset, locationInBody, orientationInBody);

    double angleRange[2] = { 0.0, SimTK::Pi * 2 };

    crankAngle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
    crankAngle->updCoordinate(PinJoint::Coord::RotationZ).setName("crankAngle");

    bodySet.adoptAndAppend(crankset);
    jointSet.adoptAndAppend(crankAngle);
}

void Cyclist::buildPedals()
{

    //Left Pedal
    double xx = 0.000459, yy = 0.0000624, zz = 0.0000198, xy = 0, xz = 0, yz = 0.0;
    double xoffset = 0, yoffset = -CRANK_LENGTH, zoffset = -Q_FACTOR / 2.0; //yoffset = crank length, zoffset = q-factor/2

    double pedalMass = 0.106; // kg
    Vec3 pedalMassCenter(0.0, 0.0, -0.0469);
    Inertia pedalInertia = Inertia(xx, yy, zz, xy, xz, yz);

    OpenSim::Body* leftPedal = new OpenSim::Body("leftPedal", pedalMass,
        pedalMassCenter, pedalInertia);

    leftPedal->attachGeometry(new Mesh("Geometry/leftpedal.stl"));

    Vec3 locationInParent(xoffset, yoffset, zoffset), orientationInParent(0);
    Vec3 locationInBody(0), orientationInBody(0);

    PinJoint* leftPedalAngle = new PinJoint("leftPedal", bodySet.get("crankset"),
        locationInParent, orientationInParent,
        *leftPedal, locationInBody, orientationInBody);

    double angleRange[2] = { 0.0, SimTK::Pi * 2 };

    leftPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
    leftPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setName("leftPedal");

    bodySet.adoptAndAppend(leftPedal);
    jointSet.adoptAndAppend(leftPedalAngle);

    //Right Pedal
    //xx,yy,zz same for right pedal
    xoffset = 0; yoffset = CRANK_LENGTH; zoffset = Q_FACTOR / 2.0;
    //pedal mass same
    pedalMassCenter = Vec3(0.0, 0.0, 0.0469);
    //pedal inertia the same

    OpenSim::Body* rightPedal = new OpenSim::Body("rightPedal", pedalMass, pedalMassCenter, pedalInertia);

    rightPedal->attachGeometry(new Mesh("Geometry/rightpedal.stl"));

    locationInParent = Vec3(xoffset, yoffset, zoffset);

    PinJoint* rightPedalAngle = new PinJoint("rightPedal", bodySet.get("crankset"),
        locationInParent, orientationInParent, *rightPedal, locationInBody, orientationInBody);

    rightPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
    rightPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setName("rightPedal");

    bodySet.adoptAndAppend(rightPedal);
    jointSet.adoptAndAppend(rightPedalAngle);
}

void Cyclist::buildSaddle()
{
    double blockMass = 20.0, blockSideLength = 0.1;
    Vec3 blockMassCenter(0);
    Inertia blockInertia = blockMass * Inertia::brick(blockSideLength,
        blockSideLength, blockSideLength);

    // Create a new block body with the specified properties
    OpenSim::Body* saddle = new OpenSim::Body("saddle", blockMass,
        blockMassCenter, blockInertia);

    // Add display geometry to the block to visualize in the GUI
    saddle->attachGeometry(new Mesh("Geometry/saddle.stl"));

    Vec3 locationInParent(-SADDLE_X_OFFSET, SADDLE_Y_OFFSET, 0.0), orientationInParent(0);
    Vec3 locationInBody(0), orientationInBody(0);

    WeldJoint* saddleToGround = new WeldJoint("saddle", model.updGround(),
        locationInParent, orientationInParent,
        *saddle, locationInBody, orientationInBody);

    bodySet.adoptAndAppend(saddle);
    jointSet.adoptAndAppend(saddleToGround);
}

void Cyclist::buildCockpit()
{
    double xx = 0.0296, yy = 0.0305, zz = 0.000361, xy = 0.000656, xz = 0, yz = 0.0;
    double xoffset = STEM_CLAMP_X_OFFSET, yoffset = STEM_CLAMP_Y_OFFSET, zoffset = 0.0; //yoffset = crank length, zoffset = q-factor/2

    double cockpitMass = 0.644; // kg
    Vec3 cockpitMassCenter(0.644, -0.0319, 0.0);
    Inertia cockpitInertia = Inertia(xx, yy, zz, xy, xz, yz);

    OpenSim::Body* cockpit = new OpenSim::Body("cockpit", cockpitMass,
        cockpitMassCenter, cockpitInertia);

    cockpit->attachGeometry(new Mesh("Geometry/cockpit.stl"));

    Vec3 locationInParent(xoffset, yoffset, zoffset), orientationInParent(0);
    Vec3 locationInBody(0), orientationInBody(0);

    WeldJoint* cockpitJoint = new WeldJoint("cockpit", model.updGround(),
        locationInParent, orientationInParent,
        *cockpit, locationInBody, orientationInBody);

    bodySet.adoptAndAppend(cockpit);
    jointSet.adoptAndAppend(cockpitJoint);
}

void Cyclist::buildBike()
{
    buildBB();
    buildCrankset();
    buildPedals();
    buildSaddle();
    buildCockpit();
}