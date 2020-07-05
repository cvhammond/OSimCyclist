#pragma once

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

class EasyModel {

	
public:
    Model model;
    BodySet bodySet;
    JointSet jointSet;
    ConstraintSet constraintSet;
    ForceSet forceSet;
    std::string name;

    EasyModel(std::string name);
    void addBody(OpenSim::Body* body);
    void addJoint(OpenSim::Joint* joint);
    void addConstraint(OpenSim::Constraint* constraint);
    void addForce(OpenSim::Force* force);
    void addBodiesFromModel(Model* model, std::list<std::string> bodyNames);
    void addJointsFromModel(Model* model, std::list<std::string> jointNames);
    void addConstraintsFromModel(Model* model, std::list<std::string> constraintNames);
    void addForcesFromModel(Model* model, std::list<std::string> forceNames);
    void addWeldJoint(std::string name, std::string parentName, std::string childName, Vec3& locationInParent = Vec3(0), Vec3& orientationInParent = Vec3(0), Vec3& locationInChild = Vec3(0), Vec3& orientationInChild = Vec3(0));
    void addClampedConstr(std::string jointName, std::string coordinateName, bool value);

    TransformAxis& makeTransformAxis(std::string name, Vec3 axis, OpenSim::Function &func);

    Vec3 getOffset(Model* model, std::string childName, std::string parentName);

    void buildModel();
    void exportModel(std::string filename);
};