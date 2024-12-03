#include <pgmspace.h>

#define SECRET
#define THINGNAME ""                    // Change data

const char WIFI_SSID[] = "";            // Change data
const char WIFI_PASSWORD[] = "";        // Change data
const char AWS_IOT_ENDPOINT[] = "";     // Change data
const char DOCK_ID[] = "DOCK00001";     // Change data

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(

)EOF";

// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(


)KEY";

// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(

)KEY";
