#ifndef VEINS_INET_CONSTANT_H_
#define VEINS_INET_CONSTANT_H_

#include <string>
using namespace std;

namespace RSUConstants {
    static constexpr const char* RSU_IDENTIFIER = "RSU";

    namespace Thresholds {
        static constexpr int B1B2_CONGESTION = 10;
        static constexpr double STUCK_TIME = 5.0;
        static constexpr double MIN_MOVEMENT = 1.0;
        static constexpr double CLEANUP_TIMEOUT = 10.0;
        static constexpr double CHECK_INTERVAL = 1.0;
    }


    inline string getRsuIdentifier(long id) {
        return string(RSU_IDENTIFIER) + to_string(id);
    }
}

#endif /* VEINS_INET_CONSTANT_H_ */
