#ifndef VEINS_VEINS_H
#define VEINS_VEINS_H

// Đảm bảo log được hiển thị đúng cách trong OMNeT++
#include <omnetpp.h>
using namespace omnetpp;

// Định nghĩa namespace veins
namespace veins {
    // Khai báo các lớp quan trọng
    class BaseFrame1609_4;
    class TraCIDemoRSU11p;
    class TraCIDemo11p;
    class DemoSafetyMessage;
    class DemoServiceAdvertisment;
    
    // Khai báo namespace TraCIMobilityAccess
    namespace TraCIMobilityAccess {
        class get;
    }
}

#endif // VEINS_VEINS_H 