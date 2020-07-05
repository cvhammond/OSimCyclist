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
std::string NAME = "Cyclist"; //name of model in OpenSim GUI
std::string FILE_NAME = "cyclist.osim"; // name of file
std::string CREDITS = "Jason Hammond"; // credit line in model

// Bike Values
double BB_WIDTH = 0.0865; //86.5mm from left bb hole to right
double Q_FACTOR = 0.146; //146mm from outside of right crank arm to left arm
double PEDAL_WIDTH = 0.053; //53mm from crank arm to center of pedal
double CRANK_LENGTH = 0.170; // 170mm crank length
double SADDLE_X_OFFSET = 0.20; //20cm horizontal offset from bottom bracket to estimated seating position
double SADDLE_Y_OFFSET = 0.65; //69cm vertical offset from bottom bracket to estimated seating position
double SADDLE_TO_PELVIS_Y_OFFSET = 0.22; // vertical offset between saddle and torso axes
double SADDLE_TO_PELVIS_X_OFFSET = 0.0; // horizontal offset between saddle and torso axes
double STEM_CLAMP_X_OFFSET = 0.40; // horizontal offset between BB and stem clamp
double STEM_CLAMP_Y_OFFSET = 0.65; // vertical offset between BB and stem clamp

Vec3 HAND_POINT_CONSTRAINT_OFFSET = Vec3(0.08, 0.035, 0.23);

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
double LEFT_HIP_JOINT_RANGE[2] = { 0.0, 3.0 * Pi / 6.0 };
double RIGHT_HIP_JOINT_RANGE[2] = { 0.0, 3.0 * Pi / 6.0 };
double PELVIS_LIST_RANGE[2] = { -Pi / 8.0, Pi / 8.0 };

// Human Values

void addModelSettings(Model* model) {
    model->setName(NAME);
    model->set_credits(CREDITS);
}

void addBodiesFromGait2392(Model* model, BodySet* bodies) {
    model->addBody(&bodies->get("pelvis"));
    model->addBody(&bodies->get("femur_r"));
    model->addBody(&bodies->get("femur_l"));
    //model->addBody(&bodies->get("torso"));
    model->addBody(&bodies->get("tibia_r"));
    model->addBody(&bodies->get("tibia_l"));
    model->addBody(&bodies->get("talus_r"));
    model->addBody(&bodies->get("talus_l"));
    model->addBody(&bodies->get("calcn_r"));
    model->addBody(&bodies->get("calcn_l"));
    model->addBody(&bodies->get("toes_r"));
    model->addBody(&bodies->get("toes_l"));

}

void addJointsFromGait2392(Model* model, JointSet* joints) {
    model->addJoint(&joints->get("ground_pelvis"));
    model->addJoint(&joints->get("back"));
    model->addJoint(&joints->get("hip_r"));
    model->addJoint(&joints->get("hip_l"));
    model->addJoint(&joints->get("knee_r"));
    model->addJoint(&joints->get("knee_l"));
    model->addJoint(&joints->get("ankle_r"));
    model->addJoint(&joints->get("ankle_l"));
    //model->addJoint(&joints->get("subtalar_r"));
    //model->addJoint(&joints->get("subtalar_l"));

}

Vec3 getOffset(Model* model, std::string childName, std::string parentName, State* state) {
    return model->getBodySet().get(childName).getPositionInGround(*state) - model->getBodySet().get(parentName).getPositionInGround(*state);
}

void addGait2392NewJoints(Model* model, Model* gait2392, State* state) {
    WeldJoint* rightToes = new WeldJoint("mtp_r", model->getBodySet().get("calcn_r"), getOffset(gait2392, "toes_r", "calcn_r", state), Vec3(0), model->getBodySet().get("toes_r"), Vec3(0), Vec3(0));
    WeldJoint* leftToes = new WeldJoint("mtp_l", model->getBodySet().get("calcn_l"), getOffset(gait2392, "toes_l", "calcn_l", state), Vec3(0), model->getBodySet().get("toes_l"), Vec3(0), Vec3(0));

    model->addJoint(rightToes);
    model->addJoint(leftToes);

    WeldJoint* rightFoot = new WeldJoint("subtalar_r", model->getBodySet().get("talus_r"), getOffset(gait2392, "calcn_r", "talus_r", state), Vec3(0), model->getBodySet().get("calcn_r"), Vec3(0), Vec3(0));
    WeldJoint* leftFoot = new WeldJoint("subtalar_l", model->getBodySet().get("talus_l"), getOffset(gait2392, "calcn_l", "talus_l", state), Vec3(0), model->getBodySet().get("calcn_l"), Vec3(0), Vec3(0));

    model->addJoint(rightFoot);
    model->addJoint(leftFoot);

}

void addGait2392JointConstraints(JointSet* joints) {
    //clamped constraints - joints cannot move outside range
    
    joints->get("knee_r").updCoordinate().setDefaultClamped(true);
    joints->get("ankle_r").updCoordinate().setDefaultClamped(true);
    joints->get("hip_r").upd_coordinates(0).setDefaultClamped(true);
    joints->get("knee_l").updCoordinate().setDefaultClamped(true);
    joints->get("ankle_l").updCoordinate().setDefaultClamped(true);
    joints->get("hip_l").upd_coordinates(0).setDefaultClamped(true);

    //default value
    joints->get("knee_r").updCoordinate().setDefaultValue(RIGHT_KNEE_DEFAULT);
    joints->get("ankle_r").updCoordinate().setDefaultValue(RIGHT_ANKLE_DEFAULT);
    joints->get("hip_r").upd_coordinates(0).setDefaultValue(RIGHT_HIP_DEFAULT);
    joints->get("knee_l").updCoordinate().setDefaultValue(LEFT_KNEE_DEFAULT);
    joints->get("ankle_l").updCoordinate().setDefaultValue(LEFT_ANKLE_DEFAULT);
    joints->get("hip_l").upd_coordinates(0).setDefaultValue(LEFT_HIP_DEFAULT);
    joints->get("ground_pelvis").upd_coordinates(0).setDefaultValue(PELVIS_TILT_DEFAULT);
    joints->get("back").upd_coordinates(0).setDefaultValue(LUMBAR_EXTENSION_DEFAULT);
    

    //range of motion constraints
    
    joints->get("ankle_r").updCoordinate().setRange(RIGHT_ANKLE_JOINT_RANGE);
    joints->get("ankle_l").updCoordinate().setRange(LEFT_ANKLE_JOINT_RANGE);

    
    joints->get("knee_r").updCoordinate().setRange(RIGHT_KNEE_JOINT_RANGE);
    joints->get("knee_l").updCoordinate().setRange(LEFT_KNEE_JOINT_RANGE);

    
    joints->get("hip_r").upd_coordinates(0).setRange(RIGHT_HIP_JOINT_RANGE);
    joints->get("hip_l").upd_coordinates(0).setRange(LEFT_HIP_JOINT_RANGE);

    
    joints->get("ground_pelvis").upd_coordinates(1).setRange(PELVIS_LIST_RANGE);
    //joints->get("ground_pelvis").upd_coordinates(2).setRange(pelvicListRange);


    //locked joints
    joints->get("ground_pelvis").upd_coordinates(0).setDefaultLocked(true);
    //joints->get("ground_pelvis").upd_coordinates(1).setDefaultLocked(true);
    joints->get("ground_pelvis").upd_coordinates(2).setDefaultLocked(true);
    std::cout << joints->get("ground_pelvis").upd_coordinates(2).getName() << std::endl;
}

void addGait2392(Model* model) {
    Model gait2392("gait2392_simbody.osim");
    gait2392.buildSystem();
    State& state = gait2392.initializeState();

    BodySet* bodies = gait2392.getBodySet().clone();
    addBodiesFromGait2392(model, bodies);

    JointSet* joints = gait2392.getJointSet().clone();
    
    addJointsFromGait2392(model, joints);

    addGait2392NewJoints(model, &gait2392, &state);

    addGait2392JointConstraints(joints);

    ForceSet* forces = gait2392.getForceSet().clone();
}

void addBodiesFromArmModel(Model* model, BodySet* bodies) {
    model->addBody(&bodies->get("torso"));
    model->addBody(&bodies->get("humerus_r"));
    model->addBody(&bodies->get("ulna_r"));
    model->addBody(&bodies->get("radius_r"));
    model->addBody(&bodies->get("hand_r"));
    model->addBody(&bodies->get("humerus_l"));
    model->addBody(&bodies->get("ulna_l"));
    model->addBody(&bodies->get("radius_l"));
    model->addBody(&bodies->get("hand_l"));
    
}

void addJointsFromArmModel(Model* model, JointSet* joints) {
    model->addJoint(&joints->get("acromial_r"));
    model->addJoint(&joints->get("elbow_r"));
    model->addJoint(&joints->get("radioulnar_r"));
    model->addJoint(&joints->get("radius_hand_r"));
    model->addJoint(&joints->get("acromial_l"));
    model->addJoint(&joints->get("elbow_l"));
    model->addJoint(&joints->get("radioulnar_l"));
    model->addJoint(&joints->get("radius_hand_l"));
}

void addArmModelNewJoints(Model* model, Model* gait2392, State* state) {

}

void addArmModelJointConstraints(JointSet* joints) {

    joints->get("acromial_r").upd_coordinates(2).setDefaultValue(ARM_ROTATION_DEFAULT);
    joints->get("acromial_l").upd_coordinates(2).setDefaultValue(ARM_ROTATION_DEFAULT);

    joints->get("acromial_r").upd_coordinates(1).setDefaultLocked(true);
    joints->get("acromial_l").upd_coordinates(1).setDefaultLocked(true);

    joints->get("elbow_r").updCoordinate().setDefaultValue(ELBOW_DEFAULT);
    joints->get("elbow_l").updCoordinate().setDefaultValue(ELBOW_DEFAULT);
}

void addArms(Model* model) {
    
}

void addConstraints(Model* model) {
    WeldConstraint* rightCleat = new WeldConstraint("rightCleat", model->getBodySet().get("toes_r"), Vec3(0, -0.02, -PEDAL_WIDTH), model->getBodySet().get("rightPedal"), Vec3(0));
    WeldConstraint* leftCleat = new WeldConstraint("leftCleat", model->getBodySet().get("toes_l"), Vec3(0, -0.02, PEDAL_WIDTH), model->getBodySet().get("leftPedal"), Vec3(0));
    //WeldConstraint* saddle = new WeldConstraint("saddle", model->getBodySet().get("saddle"), Vec3(0, SADDLE_TO_PELVIS_OFFSET, 0), model->getBodySet().get("torso"), Vec3(0));
    PointConstraint* saddle = new PointConstraint(model->getBodySet().get("saddle"), Vec3(SADDLE_TO_PELVIS_X_OFFSET, SADDLE_TO_PELVIS_Y_OFFSET, 0), model->getBodySet().get("torso"), Vec3(0));
    //PointConstraint* saddle = new PointConstraint(model->getBodySet().get("saddle"), Vec3(0), model->getBodySet().get("pelvis"), Vec3(0));

    PointConstraint* rightHand = new PointConstraint(model->getBodySet().get("hand_r"), Vec3(0), model->getBodySet().get("cockpit"), HAND_POINT_CONSTRAINT_OFFSET);
    PointConstraint* leftHand = new PointConstraint(model->getBodySet().get("hand_l"), Vec3(0), model->getBodySet().get("cockpit"), HAND_POINT_CONSTRAINT_OFFSET.elementwiseMultiply(Vec3(1, 1, -1.0)));

    model->addConstraint(rightCleat);
    model->addConstraint(leftCleat);
    model->addConstraint(saddle);
    model->addConstraint(rightHand);
    model->addConstraint(leftHand);
}

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
    double xoffset = 0, yoffset = -CRANK_LENGTH, zoffset = -Q_FACTOR / 2.0; //yoffset = crank length, zoffset = q-factor/2

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

    //Right Pedal
    //xx,yy,zz same for right pedal
    xoffset = 0; yoffset = CRANK_LENGTH; zoffset = Q_FACTOR / 2.0;
    //pedal mass same
    pedalMassCenter = Vec3(0.0, 0.0, 0.0469);
    //pedal inertia the same

    OpenSim::Body* rightPedal = new OpenSim::Body("rightPedal", pedalMass, pedalMassCenter, pedalInertia);

    rightPedal->attachGeometry(new Mesh("Geometry/rightpedal.stl"));

    locationInParent = Vec3(xoffset, yoffset,zoffset);

    PinJoint* rightPedalAngle = new PinJoint("rightPedal", model->getBodySet().get("crankset"),
        locationInParent, orientationInParent, *rightPedal, locationInBody, orientationInBody);

    rightPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setRange(angleRange);
    rightPedalAngle->updCoordinate(PinJoint::Coord::RotationZ).setName("rightPedal");

    model->addBody(rightPedal);
    model->addJoint(rightPedalAngle);
}

void buildSaddle(Model* model) {
    double blockMass = 20.0, blockSideLength = 0.1;
    Vec3 blockMassCenter(0);
    Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, 
        blockSideLength, blockSideLength);

    // Create a new block body with the specified properties
    OpenSim::Body *saddle = new OpenSim::Body("saddle", blockMass, 
        blockMassCenter, blockInertia);

    // Add display geometry to the block to visualize in the GUI
    saddle->attachGeometry(new Mesh("Geometry/saddle.stl"));

    Vec3 locationInParent(-SADDLE_X_OFFSET, SADDLE_Y_OFFSET, 0.0), orientationInParent(0);
    Vec3 locationInBody(0), orientationInBody(0);

    WeldJoint* saddleToGround = new WeldJoint("saddle", model->updGround(),
        locationInParent, orientationInParent,
        *saddle, locationInBody, orientationInBody);

    model->addBody(saddle);
    model->addJoint(saddleToGround);

}

void buildCockpit(Model* model) {
    
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

    WeldJoint* cockpitJoint = new WeldJoint("cockpit", model->updGround(),
        locationInParent, orientationInParent,
        *cockpit, locationInBody, orientationInBody);

    model->addBody(cockpit);
    model->addJoint(cockpitJoint);
}

void addBike(Model* model) {
    buildBB(model);
    buildCrankset(model);
    buildPedals(model);
    buildSaddle(model);
    buildCockpit(model);

}

void makeSimpleCrankMotion(Model* model) {
    State& state = model->initSystem();
    // init storage
    Array<std::string> columnLabels;
    Array<std::string> stateNames = model->getStateVariableNames();
    int ny = stateNames.getSize();
    Storage* storage = new Storage(512, "states");
    columnLabels.setSize(0);
    columnLabels.append("time");
    for (int i = 0; i < ny; i++) columnLabels.append(stateNames[i]);
    storage->setColumnLabels(columnLabels);

    for (int i = 0; i < 360; i++) {
        state.setTime(i / 3600.0);
        model->getJointSet().get("crankset").getCoordinate().setValue(state, (double)i * Pi / 180.0);
        SimTK::Vector stateValues = model->getStateVariableValues(state);
        StateVector vec;
        vec.setStates(state.getTime(), stateValues);
        storage->append(vec);
    }
    STOFileAdapter_<double>::write(storage->exportToTable(), "crankangle_states.sto");
}

void addHuman(Model* model) {
    Model gait2392("gait2392_simbody.osim");
    gait2392.buildSystem();
    State& legState = gait2392.initializeState();

    Model armModel("FullBodyModel_SimpleArms_Hamner2010_Markers_v2_0.osim");
    armModel.buildSystem();
    State& armState = armModel.initializeState();

    BodySet* legBodies = gait2392.getBodySet().clone();
    addBodiesFromGait2392(model, legBodies);

    BodySet* armBodies = armModel.getBodySet().clone();
    addBodiesFromArmModel(model, armBodies);

    JointSet* legJoints = gait2392.getJointSet().clone();
    addJointsFromGait2392(model, legJoints);

    JointSet* armJoints = armModel.getJointSet().clone();
    addJointsFromArmModel(model, armJoints);

    addGait2392NewJoints(model, &gait2392, &legState);

    addArmModelNewJoints(model, &armModel, &armState);

    addGait2392JointConstraints(legJoints);

    addArmModelJointConstraints(armJoints);

    ForceSet* forces = gait2392.getForceSet().clone();

}

//int main()
//{
//    try {
//        
//        Model model;
//
//        addModelSettings(&model);
//        addHuman(&model);
//        addBike(&model);
//        addConstraints(&model);
//
//        model.finalizeConnections();
//        model.print(FILE_NAME);
//
//        makeSimpleCrankMotion(&model);
//        
//    }
//    catch (OpenSim::Exception ex)
//    {
//        std::cout << ex.getMessage() << std::endl;
//        return 1;
//    }
//    catch (SimTK::Exception::Base ex)
//    {
//        std::cout << ex.getMessage() << std::endl;
//        return 1;
//    }
//    catch (std::exception ex)
//    {
//        std::cout << ex.what() << std::endl;
//        return 1;
//    }
//    catch (...)
//    {
//        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
//        return 1;
//    }
//    std::cout << "OpenSim completed successfully" << std::endl;
//    return 0;
// } 