//==============================================================================
//The OpenSim Main header must be included in all files
#include <OpenSim/OpenSim.h>
// Set the namespace to shorten the declarations
// Note: Several classes appear in both namespaces and require using the full name
using namespace OpenSim;
using namespace SimTK;
//______________________________________________________________________________
/**
 *
 */



// Model Values
const std::string NAME = "Cyclist"; //name of model in OpenSim GUI
const std::string FILE_NAME = "cyclist.osim"; // name of file

// Bike Values
const double BBWIDTH = 0.0895; //89.5mm

// Human Values

void buildBB(Model* model) {
    Ground& ground = model->updGround();

    ground.attachGeometry(new Mesh("Geometry/bottombracket.stl"));
}

void buildCrankset(Model* model) {

    double xx = 0.00980, yy = 0.00547, zz = 0.00829, xy = 0, xz = 0, yz = 0.00254;

    double cranksetMass = 1.15;
    Vec3 cranksetMassCenter(0.0, 0.0, 0.0199);
    Inertia cranksetInertia = Inertia(xx, yy, zz, xy, xz, yz);
    OpenSim::Body* crankset = new OpenSim::Body("crankset", cranksetMass,
        cranksetMassCenter, cranksetInertia);

    crankset->attachGeometry(new Mesh("Geometry/crankset.stl"));

    Vec3 locationInParent(0), orientationInParent(0, Pi, 0);
    Vec3 locationInBody(0), orientationInBody(0, Pi, 0);
    Ground& ground = model->updGround();

    PinJoint* crankAngle = new PinJoint("crankset", ground,
        locationInParent, orientationInParent,
        *crankset, locationInBody, orientationInBody);

    double angleRange[2] = { 0.0, SimTK::Pi * 2 };

    crankAngle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
    crankAngle->updCoordinate(PinJoint::Coord::RotationZ).setName("crankAngle");

    model->addBody(crankset);
    model->addJoint(crankAngle);

}

void buildPedals(Model* model) {

    //Left Pedal
    double xx = 0.000459, yy = 0.0000624, zz = 0.0000198, xy = 0, xz = 0, yz = 0.0;
    double xoffset = 0, yoffset = -0.170, zoffset = -0.146 / 2.0; //yoffset = crank length, zoffset = q-factor/2

    double pedalMass = 0.106; // kg
    Vec3 pedalMassCenter(0.0, 0.0, -0.0469);
    Inertia pedalInertia = Inertia(xx, yy, zz, xy, xz, yz);

    OpenSim::Body* leftPedal = new OpenSim::Body("leftPedal", pedalMass,
        pedalMassCenter, pedalInertia);

    leftPedal->attachGeometry(new Mesh("Geometry/leftpedal.stl"));

    Vec3 locationInParent(xoffset, yoffset,zoffset), orientationInParent(0);
    Vec3 locationInBody(0), orientationInBody(0);

    PinJoint* leftPedalAngle = new PinJoint("leftPedal", model->getBodySet().get("crankset"),
        locationInParent, orientationInParent,
        *leftPedal, locationInBody, orientationInBody);

    double angleRange[2] = { 0.0, SimTK::Pi * 2 };

    leftPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
    leftPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setName("leftPedal");

    model->addBody(leftPedal);
    model->addJoint(leftPedalAngle);

}

void buildBike(Model* model) {

    buildBB(model);
    buildCrankset(model);
    buildPedals(model);
    

}

void buildHuman(Model* model) {

}




int main()
{
    try {
        Model model;
        model.setName(NAME);

        buildBike(&model);
        buildHuman(&model);

        model.finalizeConnections();
        model.print(FILE_NAME);
        
    }
    catch (OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (SimTK::Exception::Base ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }
    std::cout << "OpenSim completed successfully" << std::endl;
    return 0;
 } 