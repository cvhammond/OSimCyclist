//==============================================================================
//The OpenSim Main header must be included in all files
#include <OpenSim/OpenSim.h>
#include "EasyModel.h"
#include "Cyclist.h"
// Set the namespace to shorten the declarations
// Note: Several classes appear in both namespaces and require using the full name
using namespace OpenSim;
using namespace SimTK;


//______________________________________________________________________________
/**
 *
 */

class Test2 {
public:
    Test2() = default;
    Test2(String str) {
        this->str = str;
    }
    void setString(String str) {
        this->str = str;
    }
    String getString() {
        return str;
    }
private:
    String str;
};

int main() {

    Cyclist* c = new Cyclist("Cyclist");

    /*EasyModel* em = new EasyModel("Test1");

    Model gait2392("gait2392_simbody.osim");
    Model armModel("FullBodyModel_SimpleArms_Hamner2010_Markers_v2_0.osim");

    std::list<std::string> bodyNames{ "pelvis", "femur_r", "tibia_r", "talus_r", "calcn_r", "toes_r", "femur_l", "tibia_l",  "talus_l",  "calcn_l",  "toes_l"};
    em->addBodiesFromModel(&gait2392, bodyNames);

    std::list<std::string> armBodyNames{ "torso", "humerus_r", "ulna_r", "radius_r", "hand_r", "humerus_l", "ulna_l", "radius_l", "hand_l" };
    em->addBodiesFromModel(&armModel, armBodyNames);

    std::list<std::string> jointNames{"ground_pelvis", "hip_r", "knee_r", "ankle_r", "hip_l", "knee_l", "ankle_l", "back"};
    em->addJointsFromModel(&gait2392, jointNames);

    std::list<std::string> armJointNames{ "acromial_r", "elbow_r", "radioulnar_r", "radius_hand_r", "acromial_l", "elbow_l", "radioulnar_l", "radius_hand_l" };
    em->addJointsFromModel(&armModel, armJointNames);

    em->addWeldJoint("mtp_r", "calcn_r", "toes_r", em->getOffset(&gait2392, "toes_r", "calcn_r"));
    em->addWeldJoint("mtp_l", "calcn_l", "toes_l", em->getOffset(&gait2392, "toes_l", "calcn_l"));
    em->addWeldJoint("subtalar_r", "talus_r", "calcn_r", em->getOffset(&gait2392, "calcn_r", "talus_r"));
    em->addWeldJoint("subtalar_l", "talus_l", "calcn_l", em->getOffset(&gait2392, "calcn_l", "talus_l"));

    em->buildModel();
    em->exportModel("test2.osim");*/
    try{
        c->buildBike();
        c->buildHuman();
        c->buildModel();
        c->exportModel("test3.osim");
        c->makeSimpleCrankMotion("crankMotion.sto");

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

