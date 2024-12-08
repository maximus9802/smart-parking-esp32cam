#include <pgmspace.h>

#define SECRET
#define THINGNAME "ESP32CAM"                    // Change data

const char WIFI_SSID[] = "";            // Change data
const char WIFI_PASSWORD[] = "";        // Change data
const char AWS_IOT_ENDPOINT[] = "a1e4360w212q2k-ats.iot.ap-southeast-1.amazonaws.com";     // Change data
const int SLOT_ID = 2;    // Change data

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(

)EOF";

// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(


)KEY";

// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(

)KEY";
