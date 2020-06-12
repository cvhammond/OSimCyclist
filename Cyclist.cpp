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


int main()
{
    try {
        Model osimModel;
        osimModel.setName("Cyclist");

        Ground& ground = osimModel.updGround();

        ground.attachGeometry(new Mesh("Geometry/chainring.stl"));
        //ground.attachGeometry(new Mesh("anchor1.vtp"));
        //ground.attachGeometry(new Mesh("anchor2.vtp"));

        double blockMass = 20.0, blockSideLength = 0.1;
        Vec3 blockMassCenter(0);
        Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, 
            blockSideLength, blockSideLength);

        OpenSim::Body *rightCrankArm = new OpenSim::Body("crank", blockMass, 
            blockMassCenter, blockInertia);

        rightCrankArm->attachGeometry(new Mesh("Geometry/crankarm.stl"));

        double halfLength = blockSideLength/2.0;
        
        Vec3 locationInParent(0), orientationInParent(0);
        Vec3 locationInBody(0), orientationInBody(0);
        std::cout << locationInBody << std::endl;
        PinJoint *crank_angle = new PinJoint("crankarm", ground, 
            locationInParent, orientationInParent, 
            *rightCrankArm, locationInBody, orientationInBody);

        double angleRange[2] = { 0.0, SimTK::Pi * 2};
        
        crank_angle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
        
        osimModel.addBody(rightCrankArm);
        osimModel.addJoint(crank_angle);

        /*OpenSim::Body* block2 = new OpenSim::Body("block2", blockMass,
            blockMassCenter, blockInertia);

        block2->attachGeometry(new Mesh("chainring.stl"));

        Vec3 locationInParent2(0), orientationInParent2(0);
        Vec3 locationInBody2(0), orientationInBody2(0);
        std::cout << locationInBody << std::endl;
        FreeJoint* block2Toblock1 = new FreeJoint("block2Toblock1", *rightCrankArm,
            locationInParent2, orientationInParent2,
            *block2, locationInBody2, orientationInBody2);

        

        osimModel.addBody(block2);
        osimModel.addJoint(block2Toblock1);*/

        osimModel.finalizeConnections();

        osimModel.print("Cyclist.osim");
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