#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <glm/glm.hpp>

namespace Config {

    const std::string dataDir = "./data";


    // Walk cycle
    const std::string asfFile = dataDir + "/08.asf";
    const std::string amcFile = dataDir + "/08_01_cycle.amc";
    const glm::vec3 basePosition(0.421534, 0, -0.24297);
    const glm::vec3 baseVelocity(-0.0221038, 0.0296905, 1.55497);

    /*
    // Original walking animation
    const std::string asfFile = dataDir + "/08.asf";
    const std::string amcFile = dataDir + "/08_01.amc";
    const glm::vec3 basePosition(0,0,0);
    const glm::vec3 baseVelocity(0,0,0);
    */

    // Macarena
    /*
    const std::string asfFile = dataDir + "/143.asf";
    const std::string amcFile = dataDir + "/143_35.amc";
    const glm::vec3 basePosition(0,0,0);
    const glm::vec3 baseVelocity(0,0,0);
    */

}

#endif
