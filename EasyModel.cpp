#pragma once
#include "EasyModel.h"

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

EasyModel::EasyModel(std::string name)
{
    this->name = name;
}

void EasyModel::addBody(OpenSim::Body* body)
{
    bodySet.cloneAndAppend(*body);
}

void EasyModel::addJoint(OpenSim::Joint* joint)
{
    jointSet.cloneAndAppend(*joint);
}

void EasyModel::addConstraint(OpenSim::Constraint* constraint)
{
    constraintSet.cloneAndAppend(*constraint);
}

void EasyModel::addForce(OpenSim::Force* force)
{
    forceSet.cloneAndAppend(*force);
}

void EasyModel::addBodiesFromModel(Model* model, std::list<std::string> bodyNames)
{
    BodySet* bodies = model->getBodySet().clone();
    for (std::string s : bodyNames) {
        addBody(&(bodies->get(s)));
    }
}

void EasyModel::addJointsFromModel(Model* model, std::list<std::string> jointNames)
{
    JointSet* joints = model->getJointSet().clone();
    for (std::string s : jointNames) {
        addJoint(&(joints->get(s)));
    }
}

void EasyModel::addConstraintsFromModel(Model* model, std::list<std::string> constraintNames)
{
    ConstraintSet* constraints = model->getConstraintSet().clone();
    for (std::string s : constraintNames) {
        addConstraint(&(constraints->get(s)));
    }
}

void EasyModel::addForcesFromModel(Model* model, std::list<std::string> forceNames)
{
    ForceSet* forces = model->getForceSet().clone();
    for (std::string s : forceNames) {
        addForce(&(forces->get(s)));
    }
}

void EasyModel::addWeldJoint(std::string name, std::string parentName, std::string childName, Vec3& locationInParent, Vec3& orientationInParent, Vec3& locationInChild, Vec3& orientationInChild)
{

    WeldJoint *temp = new WeldJoint(name, bodySet.get(parentName), locationInParent, orientationInParent, bodySet.get(childName), locationInChild, orientationInChild);
    jointSet.adoptAndAppend(temp);
}

void EasyModel::addClampedConstr(std::string jointName, std::string coordinateName, bool value)
{
    for (int i = 0; i < jointSet.get(jointName).countNumComponents(); i++) {
        if (jointSet.get(jointName).upd_coordinates(i).getName() == coordinateName) {
            jointSet.get(jointName).upd_coordinates(i).setDefaultClamped(value);
            break;
        }
    }
}

TransformAxis& EasyModel::makeTransformAxis(std::string name, Vec3 axis, OpenSim::Function &func)
{
    Array<std::string>* names = new Array<std::string>("T", 1);
    names->set(0, name);
    TransformAxis* tempAxis = new TransformAxis(*names, axis);
    tempAxis->setFunction(func);
    return *tempAxis;
}

Vec3 EasyModel::getOffset(Model* model, std::string childName, std::string parentName)
{
    Model *model2 = model->clone();
    State& state = model2->initSystem();
    return model2->getBodySet().get(childName).getPositionInGround(state) - model2->getBodySet().get(parentName).getPositionInGround(state);
}

void EasyModel::buildModel()
{
    for (int i=0; i < bodySet.getSize(); i++) {
        std::cout << bodySet[i] << std::endl;
        model.addBody(&bodySet[i]);
    }
    for (int i=0; i < jointSet.getSize(); i++) {
        std::cout << jointSet[i] << std::endl;
        model.addJoint(&jointSet[i]);
    }
    for (int i=0; i < constraintSet.getSize(); i++) {
        model.addConstraint(&constraintSet[i]);
    }
    for (int i=0; i < forceSet.getSize(); i++) {
        model.addForce(&forceSet[i]);
    }
    model.finalizeConnections();
    
}

void EasyModel::exportModel(std::string filename)
{
    model.setName(name);
    model.print(filename);
}
