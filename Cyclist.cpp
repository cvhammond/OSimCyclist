#include "Cyclist.h"

Cyclist::Cyclist(std::string name) : EasyModel(name) {

}


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

void Cyclist::buildHandMarkers()
{
    OpenSim::Body* rightHand = new OpenSim::Body("hand_r_position", 0.01, Vec3(0), Inertia(0));

    rightHand->attachGeometry(new Mesh("Geometry/block.vtp"));
    /*SpatialTransform* stR = new SpatialTransform();
    stR->set_translation1(makeTransformAxis("RightHandX", Vec3(1, 0, 0), *(new LinearFunction)));
    stR->set_translation2(makeTransformAxis("RightHandY", Vec3(0, 1, 0), *(new LinearFunction)));
    stR->set_translation3(makeTransformAxis("RightHandZ", Vec3(0, 0, 1), *(new LinearFunction)));
    */
    FreeJoint* rightHandJoint = new FreeJoint("ground_hand_r", model.updGround(), ORIGIN_TO_RIGHT_HAND, Vec3(0), *rightHand, Vec3(0), Vec3(0));

    std::string rightNames[6]{ "XRotRightHand", "YRotRightHand", "ZRotRightHand", "XTraRightHand", "YTraRightHand", "ZTraRightHand" };


    for (int i = 0; i < 6; i++) {
        Coordinate* coord = &rightHandJoint->upd_coordinates(i);
        coord->setName(rightNames[i]);
        coord->setDefaultLocked(true);
        coord->setDefaultClamped(true);
        if (i < 3) {
            coord->setRange(new double[2]{ -Pi,Pi });
        }
        else {
            coord->setRange(new double[2]{ -1.0, 1.0 });
        }
    }

    bodySet.adoptAndAppend(rightHand);
    jointSet.adoptAndAppend(rightHandJoint);

    OpenSim::Body* leftHand = new OpenSim::Body("hand_l_position", 0.01, Vec3(0), Inertia(0));
    
    leftHand->attachGeometry(new Mesh("Geometry/block.vtp"));

    FreeJoint* leftHandJoint = new FreeJoint("ground_hand_l", model.updGround(), ORIGIN_TO_LEFT_HAND, Vec3(0), *leftHand, Vec3(0), Vec3(0));

    std::string leftNames[6]{ "XRotLeftHand", "YRotLeftHand", "ZRotLeftHand", "XTraLeftHand", "YTraLeftHand", "ZTraLeftHand" };

    for (int i = 0; i < 6; i++) {
        Coordinate* coord = &leftHandJoint->upd_coordinates(i);
        coord->setName(leftNames[i]);
        coord->setDefaultLocked(true);
        coord->setDefaultClamped(true);
        if (i < 3) {
            coord->setRange(new double[2]{ -Pi,Pi });
        }
        else {
            coord->setRange(new double[2]{ -1.0, 1.0 });
        }
    }

    bodySet.adoptAndAppend(leftHand);
    jointSet.adoptAndAppend(leftHandJoint);
}

void Cyclist::buildSaddleMarker()
{

    OpenSim::Body* saddle = new OpenSim::Body("pelvis_on_saddle", 0.01, Vec3(0), Inertia(0));

    saddle->attachGeometry(new Mesh("Geometry/block.vtp"));

    FreeJoint* saddleJoint = new FreeJoint("saddle_pelvis", model.updGround(), ORIGIN_TO_PELVIS,  Vec3(0), *saddle, Vec3(0), Vec3(0));

    std::string names[6] { "XRotPelvisSaddle", "YRotPelvisSaddle", "ZRotPelvisSaddle", "XTraPelvisSaddle", "YTraPelvisSaddle", "ZTraPelvisSaddle" };

    for (int i = 0; i < 6; i++) {
        Coordinate* coord = &saddleJoint->upd_coordinates(i);
        coord->setName(names[i]);
        coord->setDefaultLocked(true);
        coord->setDefaultClamped(true);
        if (i < 3) {
            coord->setRange(new double[2]{ -Pi,Pi });
        }
        else {
            coord->setRange(new double[2]{ -1.0, 1.0 });
        }
    }

    bodySet.adoptAndAppend(saddle);
    jointSet.adoptAndAppend(saddleJoint);
}

void Cyclist::addBodies()
{
    Model gait2392("gait2392_simbody.osim");
    Model armModel("FullBodyModel_SimpleArms_Hamner2010_Markers_v2_0.osim");

    std::list<std::string> bodyNames{ "pelvis", "femur_r", "tibia_r", "talus_r", "calcn_r", "toes_r", "femur_l", "tibia_l",  "talus_l",  "calcn_l",  "toes_l" };
    addBodiesFromModel(&gait2392, bodyNames);

    std::list<std::string> armBodyNames{ "torso", "humerus_r", "ulna_r", "radius_r", "hand_r", "humerus_l", "ulna_l", "radius_l", "hand_l" };
    addBodiesFromModel(&armModel, armBodyNames);
}

void Cyclist::addJoints()
{
    Model gait2392("gait2392_simbody.osim");
    Model armModel("FullBodyModel_SimpleArms_Hamner2010_Markers_v2_0.osim");

    std::list<std::string> jointNames{ "ground_pelvis", "hip_r", "knee_r", "ankle_r", "hip_l", "knee_l", "ankle_l", "back" };
    addJointsFromModel(&gait2392, jointNames);

    std::list<std::string> armJointNames{ "acromial_r", "elbow_r", "radioulnar_r", "radius_hand_r", "acromial_l", "elbow_l", "radioulnar_l", "radius_hand_l" };
    addJointsFromModel(&armModel, armJointNames);

    addWeldJoint("mtp_r", "calcn_r", "toes_r", getOffset(&gait2392, "toes_r", "calcn_r"));
    addWeldJoint("mtp_l", "calcn_l", "toes_l", getOffset(&gait2392, "toes_l", "calcn_l"));
    addWeldJoint("subtalar_r", "talus_r", "calcn_r", getOffset(&gait2392, "calcn_r", "talus_r"));
    addWeldJoint("subtalar_l", "talus_l", "calcn_l", getOffset(&gait2392, "calcn_l", "talus_l"));

}

void Cyclist::addJointSettings()
{
    //clamped constraints - joints cannot move outside range
    jointSet.get("knee_r").updCoordinate().setDefaultClamped(true);
    jointSet.get("ankle_r").updCoordinate().setDefaultClamped(true);
    jointSet.get("hip_r").upd_coordinates(0).setDefaultClamped(true);
    jointSet.get("knee_l").updCoordinate().setDefaultClamped(true);
    jointSet.get("ankle_l").updCoordinate().setDefaultClamped(true);
    jointSet.get("hip_l").upd_coordinates(0).setDefaultClamped(true);

    //default value
    jointSet.get("knee_r").updCoordinate().setDefaultValue(RIGHT_KNEE_DEFAULT);
    jointSet.get("ankle_r").updCoordinate().setDefaultValue(RIGHT_ANKLE_DEFAULT);
    jointSet.get("hip_r").upd_coordinates(0).setDefaultValue(RIGHT_HIP_DEFAULT);
    jointSet.get("knee_l").updCoordinate().setDefaultValue(LEFT_KNEE_DEFAULT);
    jointSet.get("ankle_l").updCoordinate().setDefaultValue(LEFT_ANKLE_DEFAULT);
    jointSet.get("hip_l").upd_coordinates(0).setDefaultValue(LEFT_HIP_DEFAULT);
    jointSet.get("ground_pelvis").upd_coordinates(0).setDefaultValue(PELVIS_TILT_DEFAULT);
    jointSet.get("back").upd_coordinates(0).setDefaultValue(LUMBAR_EXTENSION_DEFAULT);
    jointSet.get("acromial_r").upd_coordinates(2).setDefaultValue(ARM_ROTATION_DEFAULT);
    jointSet.get("acromial_l").upd_coordinates(2).setDefaultValue(ARM_ROTATION_DEFAULT);
    jointSet.get("elbow_r").updCoordinate().setDefaultValue(ELBOW_DEFAULT);
    jointSet.get("elbow_l").updCoordinate().setDefaultValue(ELBOW_DEFAULT);

    //range of motion constraints
    jointSet.get("ankle_r").updCoordinate().setRange(RIGHT_ANKLE_JOINT_RANGE);
    jointSet.get("ankle_l").updCoordinate().setRange(LEFT_ANKLE_JOINT_RANGE);


    jointSet.get("knee_r").updCoordinate().setRange(RIGHT_KNEE_JOINT_RANGE);
    jointSet.get("knee_l").updCoordinate().setRange(LEFT_KNEE_JOINT_RANGE);


    jointSet.get("hip_r").upd_coordinates(0).setRange(RIGHT_HIP_JOINT_RANGE);
    jointSet.get("hip_l").upd_coordinates(0).setRange(LEFT_HIP_JOINT_RANGE);


    jointSet.get("ground_pelvis").upd_coordinates(1).setRange(PELVIS_LIST_RANGE);
    //joints->get("ground_pelvis").upd_coordinates(2).setRange(pelvicListRange);

    jointSet.get("acromial_r").upd_coordinates(0).setRange(RIGHT_ARM_FLEXION_RANGE);
    jointSet.get("acromial_l").upd_coordinates(0).setRange(LEFT_ARM_FLEXION_RANGE);


    //locked joints
    jointSet.get("ground_pelvis").upd_coordinates(0).setDefaultLocked(true);
    //joints->get("ground_pelvis").upd_coordinates(1).setDefaultLocked(true);
    jointSet.get("ground_pelvis").upd_coordinates(2).setDefaultLocked(true);

    jointSet.get("acromial_r").upd_coordinates(1).setDefaultLocked(false);
    jointSet.get("acromial_l").upd_coordinates(1).setDefaultLocked(false);
    //jointSet.get("acromial_r").upd_coordinates(2).setDefaultLocked(true);
    //jointSet.get("acromial_l").upd_coordinates(2).setDefaultLocked(true);
    jointSet.get("back").upd_coordinates(2).setDefaultLocked(true);
}

void Cyclist::addConstraints()
{
    WeldConstraint* rightCleat = new WeldConstraint("rightCleat", bodySet.get("toes_r"), Vec3(0, -0.02, -PEDAL_WIDTH), bodySet.get("rightPedal"), Vec3(0));
    WeldConstraint* leftCleat = new WeldConstraint("leftCleat", bodySet.get("toes_l"), Vec3(0, -0.02, PEDAL_WIDTH), bodySet.get("leftPedal"), Vec3(0));
    //WeldConstraint* saddle = new WeldConstraint("saddle", model->getBodySet().get("saddle"), Vec3(0, SADDLE_TO_PELVIS_OFFSET, 0), model->getBodySet().get("torso"), Vec3(0));
    PointConstraint* saddle = new PointConstraint(bodySet.get("saddle"), Vec3(SADDLE_TO_PELVIS_X_OFFSET, SADDLE_TO_PELVIS_Y_OFFSET, 0), bodySet.get("pelvis"), Vec3(0));
    //PointConstraint* saddle = new PointConstraint(model->getBodySet().get("saddle"), Vec3(0), model->getBodySet().get("pelvis"), Vec3(0));

    PointConstraint* rightHand = new PointConstraint(bodySet.get("cockpit"), HAND_POINT_CONSTRAINT_OFFSET, bodySet.get("hand_r"), Vec3(0));
    PointConstraint* leftHand = new PointConstraint(bodySet.get("cockpit"), HAND_POINT_CONSTRAINT_OFFSET.elementwiseMultiply(Vec3(1.0, 1.0, -1.0)), bodySet.get("hand_l"), Vec3(0));

    constraintSet.adoptAndAppend(rightCleat);
    constraintSet.adoptAndAppend(leftCleat);
    constraintSet.adoptAndAppend(saddle);
    constraintSet.adoptAndAppend(rightHand);
    constraintSet.adoptAndAppend(leftHand);
}


void Cyclist::addForces()
{
    Model gait2392("gait2392_simbody.osim");
    std::list<std::string> names;
    for (int i = 0; i < gait2392.getForceSet().getSize(); i++) {
        names.push_back(gait2392.getForceSet()[i].getName());
    }
    addForcesFromModel(&gait2392, names);
    

}

void Cyclist::buildHuman()
{
    addBodies();
    addJoints();
    addJointSettings();
    addConstraints();
    addForces();
}

void Cyclist::makeSimpleCrankMotion(std::string filename)
{
    State& state = model.initSystem();
    // init storage
    Array<std::string> columnLabels;
    Array<std::string> stateNames = model.getStateVariableNames();
    int ny = stateNames.getSize();
    Storage* storage = new Storage(512, "states");
    columnLabels.setSize(0);
    columnLabels.append("time");
    for (int i = 0; i < ny; i++) columnLabels.append(stateNames[i]);
    storage->setColumnLabels(columnLabels);

    

    for (int i = 0; i < 3600; i++) {
        state.setTime(i / 360.0);
        model.getJointSet().get("crankset").getCoordinate().setValue(state, (double)i * Pi / 180.0);
        for (int j = 0; j < model.getForceSet().getSize(); j++) {
            OpenSim::Thelen2003Muscle* m = (OpenSim::Thelen2003Muscle*) & model.getForceSet()[j];
            m->computeInitialFiberEquilibrium(state);
        }
        SimTK::Vector stateValues = model.getStateVariableValues(state);
        StateVector vec;
        vec.setStates(state.getTime(), stateValues);
        storage->append(vec);
    }
    STOFileAdapter_<double>::write(storage->exportToTable(), filename);
}


