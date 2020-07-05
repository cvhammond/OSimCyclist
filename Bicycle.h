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

class Bicycle {
public:
    void buildBB();
    void buildCrankset();
    void buildPedals();
    void buildSaddle();
    void buildCockpit();
    void buildBike();
};

