#ifndef ULISSEGEOMETRYDEFINES_H
#define ULISSEGEOMETRYDEFINES_H
#include <iostream>
#include <rml/RMLDefines.h>

namespace ulisse {
namespace robotModelID {
    const std::string ASV = "myVehicle";
}

namespace curves {
    const std::string circle_arc = "circle_arc";
    const std::string straight_line = "straight_line";
}

namespace frameAndConstraints {
    const std::string framesAndConstraints = "framesAndConstraints";
    const std::string asvTBase = "vTbase";
    const std::string translation = "translation";
    const std::string rpy = "rpy";
    const std::string wTauv = "wTauv";
    const std::string asv_active_dof_property = framesAndConstraints + "." + "vehicleActiveDof";
}
}
#endif
