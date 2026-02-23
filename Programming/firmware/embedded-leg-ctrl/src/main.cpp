#include <ServoCtrl.h>
#include <WiFi.h>
#include <zenoh-pico.h>

#if Z_FEATURE_PUBLICATION == 1
// WiFi-specific parameters
#define SSID "TP-Link_BF98"
#define PASS "59429128"
// #define SSID "iPhone de Nico "
// #define PASS "almohada"

// Zenoh-specific parameters
#define MODE "client"
#define LOCATOR "tcp/192.168.0.36:7447" // If empty, scout

#define LED_PIN 2

#define KEYEXPR "joint_commands" // ros topic
/*zenoh-bridge-ros2dds*/         // open terminal to enable bridge
/*ros2 topic pub /joint_commands std_msgs/msg/Float32MultiArray "layout:
  dim: []
  data_offset: 0
data: [0.0, 0.78, -1.57, 0.0, 0.78, -1.57, 0.0, 0.78, -1.57, 0.0, 0.78, -1.57]"
*/
// to send data to the esp32

z_owned_session_t s;      // session object
z_owned_subscriber_t sub; // subsciber object

uint8_t buffer[256]; // creem buffer de 1 byte
float leg1[3];
float leg2[3];
float leg3[3];
float leg4[3];

#define LED_PIN 2 // Pin GPIO
#define LF_HHA 16 // left front hip abductor adductor
#define LF_HFE 17 // left front hip flexor extensor
#define LF_KFE 18 // left front knee flexor extensor

const int lf_haa_channel = 0; // Canal LEDC (0-15)
const int lf_hfe_channel = 1; // Canal LEDC (0-15)
const int lf_kfe_channel = 2; // Canal LEDC (0-15)
const int freq = 50;          // Frecuencia PWM
const int resolution = 16;    // Resolución (1-16 bits)

ServoCtrl lf_haa(LF_HHA, lf_haa_channel, resolution);
ServoCtrl lf_hfe(LF_HFE, lf_hfe_channel, resolution);

void data_handler(z_loaned_sample_t *sample, void *arg)
{
  const z_loaned_bytes_t *payload = z_sample_payload(sample);
  z_bytes_reader_t reader = z_bytes_get_reader(payload);
  size_t len = z_bytes_reader_remaining(&reader);

  if (len > sizeof(buffer))
    len = sizeof(buffer);
  z_bytes_reader_read(&reader, buffer, len);
  // Imprime todo en hex para ver la estructura
  leg1[0] = *(float *)(&buffer[16]);
  leg1[1] = *(float *)(&buffer[20]);
  leg1[2] = *(float *)(&buffer[24]);

  Serial.print("Leg 1: ");
  Serial.print(leg1[0]);
  Serial.print(", ");
  Serial.print(leg1[1]);
  Serial.print(", ");
  Serial.println(leg1[2]);
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);

  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
  Serial.println("OK");
  // Initialize Zenoh Session and other parameters
  z_owned_config_t config;
  z_config_default(&config);
  zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
  if (strcmp(LOCATOR, "") != 0)
  {
    if (strcmp(MODE, "client") == 0)
    {
      zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, LOCATOR);
    }
    else
    {
      zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY, LOCATOR);
    }
  }

  // Open Zenoh session
  Serial.print("Opening Zenoh Session...");
  if (z_open(&s, z_config_move(&config), NULL) < 0)
  {
    Serial.println("Unable to open session!");
    while (1)
    {
      ;
    }
  }
  Serial.println("OK");

  // Start read and lease tasks for zenoh-pico
  if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0)
  {
    Serial.println("Unable to start read and lease tasks\n");
    z_session_drop(z_session_move(&s));
    while (1)
    {
      ;
    }
  }
  // Declare Zenoh subscriber
  Serial.print("Declaring Subscriber on ");
  Serial.print(KEYEXPR);
  Serial.println(" ...");
  z_owned_closure_sample_t callback;
  z_closure_sample(&callback, data_handler, NULL, NULL);
  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
  if (z_declare_subscriber(z_session_loan(&s), &sub, z_view_keyexpr_loan(&ke), z_closure_sample_move(&callback),
                           NULL) < 0)
  {
    Serial.println("Unable to declare subscriber.");
    while (1)
    {
      ;
    }
  }
  Serial.println("OK");
  Serial.println("Zenoh setup finished!");
  digitalWrite(LED_PIN, HIGH);

  delay(300);
}

void loop()
{
  lf_haa.write(leg1[0]);
  lf_hfe.write(leg1[1]);
  // lf_kfe.write(leg1[2]);
}
#endif