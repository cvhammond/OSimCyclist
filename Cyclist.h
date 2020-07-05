#pragma once

//==============================================================================
//The OpenSim Main header must be included in all files
#include <OpenSim/OpenSim.h>
#include "EasyModel.h"
// Set the namespace to shorten the declarations
// Note: Several classes appear in both namespaces and require using the full name
using namespace OpenSim;
using namespace SimTK;
//______________________________________________________________________________
/**
 *
 */

class Cyclist: public EasyModel {
    
public:

    Cyclist(std::string name);

    // Bike Values
    double BB_WIDTH = 0.0865; //86.5mm from left bb hole to right
    double Q_FACTOR = 0.146; //146mm from outside of right crank arm to left arm
    double PEDAL_WIDTH = 0.053; //53mm from crank arm to center of pedal
    double CRANK_LENGTH = 0.170; // 170mm crank length
    double SADDLE_X_OFFSET = 0.20; //20cm horizontal offset from bottom bracket to estimated seating position
    double SADDLE_Y_OFFSET = 0.65; //69cm vertical offset from bottom bracket to estimated seating position
    double SADDLE_TO_PELVIS_Y_OFFSET = 0.15; // vertical offset between saddle and torso axes
    double SADDLE_TO_PELVIS_X_OFFSET = 0.1; // horizontal offset between saddle and torso axes
    double STEM_CLAMP_X_OFFSET = 0.40; // horizontal offset between BB and stem clamp
    double STEM_CLAMP_Y_OFFSET = 0.65; // vertical offset between BB and stem clamp

    Vec3 HAND_POINT_CONSTRAINT_OFFSET = Vec3(0.08, 0.035, 0.23);
    Vec3 ORIGIN_TO_RIGHT_HAND = Vec3(0.5, 0.5, 0.23);
    Vec3 ORIGIN_TO_LEFT_HAND = Vec3(0.5, 0.5, -0.23);
    Vec3 ORIGIN_TO_PELVIS = Vec3(-0.5, -0.5, 0.0);

    // Human Values
    double RIGHT_KNEE_DEFAULT = -3.0 * Pi / 4.0;
    double RIGHT_ANKLE_DEFAULT = Pi / 8.0;
    double RIGHT_HIP_DEFAULT = Pi / 2.0;
    double LEFT_KNEE_DEFAULT = -3.0 * Pi / 4.0;
    double LEFT_ANKLE_DEFAULT = Pi / 8.0;
    double LEFT_HIP_DEFAULT = Pi / 2.0;
    double PELVIS_TILT_DEFAULT = -Pi / 16.0;
    double LUMBAR_EXTENSION_DEFAULT = -Pi / 4.0;
    double ARM_ROTATION_DEFAULT = Pi / 2.0;
    double ELBOW_DEFAULT = Pi / 20.0;

    double LEFT_ANKLE_JOINT_RANGE[2] = { -Pi / 4.0, Pi / 4.0 };
    double RIGHT_ANKLE_JOINT_RANGE[2] = { -Pi / 4.0, Pi / 4.0 };
    double LEFT_KNEE_JOINT_RANGE[2] = { -5.0 * Pi / 6.0, 0.0 };
    double RIGHT_KNEE_JOINT_RANGE[2] = { -5.0 * Pi / 6.0, 0.0 };
    double LEFT_HIP_JOINT_RANGE[2] = { 0.0, 4.0 * Pi / 6.0 };
    double RIGHT_HIP_JOINT_RANGE[2] = { 0.0, 4.0 * Pi / 6.0 };
    double PELVIS_LIST_RANGE[2] = { -Pi / 8.0, Pi / 8.0 };
    double RIGHT_ARM_FLEXION_RANGE[2] = { -Pi / 2.0, Pi };
    double LEFT_ARM_FLEXION_RANGE[2] = { -Pi / 2.0, Pi };

    void buildBB();
    void buildCrankset();
    void buildPedals();
    void buildSaddle();
    void buildCockpit();
    void buildBike();

    void buildHandMarkers();
    void buildSaddleMarker();

    void addBodies();
    void addJoints();
    void addJointSettings();
    void addConstraints();
    void addForces();
    void buildHuman();

    void makeSimpleCrankMotion(std::string filename);
};