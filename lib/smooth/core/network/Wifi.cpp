/*
Smooth - A C++ framework for embedded programming on top of Espressif's ESP-IDF
Copyright 2019 Per Malmberg (https://gitbub.com/PerMalmberg)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "smooth/core/network/Wifi.h"

#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi_types.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

#include <cstring>
#include <sstream>

#include "smooth/core/ipc/Publisher.h"
#include "smooth/core/logging/log.h"
#include "smooth/core/network/NetworkStatus.h"
#include "smooth/core/util/copy_min_to_buffer.h"

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
static_assert(
    CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE >= 3072,
    "Need enough stack to be able to log in the event loop callback.");
#endif

using namespace smooth::core::util;
using namespace smooth::core;

namespace smooth::core::network {
struct esp_ip4_addr Wifi::ip = {0};

Wifi::Wifi() { initialise_wifi(); }

Wifi::~Wifi() {
  esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID,
                                        instance_ip_event);
  esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        instance_wifi_event);

  esp_event_handler_instance_unregister(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID,
                                        instance_prov_event);

  esp_wifi_disconnect();
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_netif_deinit();
}

void Wifi::initialise_wifi() {
  // s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  // ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &Wifi::wifi_event_callback, this,
      &instance_wifi_event));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, &Wifi::wifi_event_callback, this,
      &instance_ip_event));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &Wifi::wifi_event_callback, this,
      &instance_prov_event));
}

void Wifi::set_host_name(const std::string& name) { host_name = name; }

void Wifi::set_ap_credentials(const std::string& wifi_ssid,
                              const std::string& wifi_password) {
  this->ssid = wifi_ssid;
  this->password = wifi_password;
  Log::info("NET::WIFI",
            " Wi-Fi credentials set"
            "\n\tSSID     : {}\n\tPassword : {}",
            this->ssid, this->password);
}

void Wifi::set_auto_connect(bool auto_connect) {
  this->auto_connect_to_ap = auto_connect;
}

void Wifi::connect_to_ap() {
  // Prepare to connect to the provided SSID and password
  wifi_config_t config;
  memset(&config, 0, sizeof(config));
  copy_min_to_buffer(ssid.begin(), ssid.length(), config.sta.ssid);
  copy_min_to_buffer(password.begin(), password.length(), config.sta.password);
  config.sta.bssid_set = false;

  // Store Wifi settings in RAM - it is the applications responsibility to store
  // settings.
  esp_wifi_set_storage(WIFI_STORAGE_RAM);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &config));

  connect();
}

bool Wifi::is_connected_to_ap() const { return connected_to_ap; }

bool Wifi::is_provisioning_ble() const { return provisioning_active; }

bool Wifi::is_should_save_creds() const { return should_save_creds; }

bool Wifi::is_prov_ended() const { return prov_ended; }

bool Wifi::is_prov_success() const { return prov_success; }

void Wifi::setCredentialsSaved() { should_save_creds = false; }

void Wifi::wifi_event_callback(void* event_handler_arg,
                               esp_event_base_t event_base, int32_t event_id,
                               void* event_data) {
  // Note: be very careful with what you do in this method - it runs under the
  // event task (sys_evt) with a very small default stack.
  Wifi* wifi = reinterpret_cast<Wifi*>(event_handler_arg);

  if (event_base == WIFI_EVENT) {
    if (event_id == WIFI_EVENT_STA_START) {
      // esp_netif_set_hostname(wifi->interface, wifi->host_name.c_str());
      Log::info("NET::WIFI::CB", "WIFI EVENT STA START");
      // esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
      Log::info("NET::WIFI::CB", "WIFI EVENT STA CONNECTED");
      wifi->connected_to_ap = true;

    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      Log::info("NET::WIFI::CB",
                "WIFI EVENT STA DISCONNECTED. Connecting to the AP again...");
      wifi->ip.addr = 0;
      wifi->connected_to_ap = false;
      publish_status(wifi->connected_to_ap, true);

      if (wifi->auto_connect_to_ap) {
        esp_wifi_stop();
        wifi->connect();
      }

    } else if (event_id == WIFI_EVENT_AP_START) {
      wifi->ip.addr = 0xC0A80401;  // 192.168.4.1
      publish_status(true, true);

    } else if (event_id == WIFI_EVENT_AP_STOP) {
      wifi->ip.addr = 0;
      Log::info("NET::WIFI::CB", "AP stopped");
      publish_status(false, true);

    } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
      auto data = reinterpret_cast<wifi_event_ap_staconnected_t*>(event_data);
      Log::info("NET::WIFI::CB",
                "Station connected. MAC: {}:{}:{}:{}:{}:{} join, AID={}",
                data->mac[0], data->mac[1], data->mac[2], data->mac[3],
                data->mac[4], data->mac[5], data->aid);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
      auto data =
          reinterpret_cast<wifi_event_ap_stadisconnected_t*>(event_data);

      Log::info("NET::WIFI::CB",
                "Station disconnected. MAC: {}:{}:{}:{}:{}:{} join, AID={}",
                data->mac[0], data->mac[1], data->mac[2], data->mac[3],
                data->mac[4], data->mac[5], data->aid);
    }
  } else if (event_base == IP_EVENT) {
    if (event_id == IP_EVENT_STA_GOT_IP || event_id == IP_EVENT_GOT_IP6 ||
        event_id == IP_EVENT_ETH_GOT_IP) {
      auto ip_changed =
          event_id == IP_EVENT_STA_GOT_IP
              ? reinterpret_cast<ip_event_got_ip_t*>(event_data)->ip_changed
              : true;
      publish_status(true, ip_changed);
      wifi->ip.addr =
          reinterpret_cast<ip_event_got_ip_t*>(event_data)->ip_info.ip.addr;
    } else if (event_id == IP_EVENT_STA_LOST_IP) {
      wifi->ip.addr = 0;
      publish_status(false, true);
    }
  } else if (event_base == WIFI_PROV_EVENT) {
    switch (event_id) {
      case WIFI_PROV_START:
        Log::info("NET::WIFI::CB", "Provisioning started");
        wifi->provisioning_active = true;
        break;
      case WIFI_PROV_CRED_RECV: {
        wifi_sta_config_t* wifi_sta_cfg = (wifi_sta_config_t*)event_data;
        Log::info("NET::WIFI::CB",
                  "Received Wi-Fi credentials"
                  "\n\tSSID     : {}\n\tPassword : {}",
                  (const char*)wifi_sta_cfg->ssid,
                  (const char*)wifi_sta_cfg->password);

        wifi->set_ap_credentials((const char*)wifi_sta_cfg->ssid,
                                 (const char*)wifi_sta_cfg->password);
        break;
      }
      case WIFI_PROV_CRED_FAIL: {
        wifi_prov_sta_fail_reason_t* reason =
            (wifi_prov_sta_fail_reason_t*)event_data;
        Log::error("NET::WIFI::CB",
                   "Provisioning failed!\n\tReason : {}"
                   "\n\tPlease reset to factory and retry provisioning",
                   (*reason == WIFI_PROV_STA_AUTH_ERROR)
                       ? "Wi-Fi station authentication failed"
                       : "Wi-Fi access-point not found");
        // try multiple times
        wifi_prov_mgr_stop_provisioning();
        break;
      }
      case WIFI_PROV_CRED_SUCCESS:
        // save credentials flag
        Log::info("NET::WIFI::CB", "Provisioning successful");
        wifi->should_save_creds = true;
        wifi->prov_success = true;
        break;
      case WIFI_PROV_END:
        /* De-initialize manager once provisioning is finished */
        Log::info("NET::WIFI::CB", "Provisioning stopped");
        wifi->provisioning_active = false;
        wifi->prov_ended = true;
        wifi_prov_mgr_deinit();
        break;
      default:
        break;
    }  // namespace smooth::core::network
  }
}

/* Handler for the optional provisioning endpoint registered by the application.
 * The data format can be chosen by applications. Here, we are using plain ascii
 * text. Applications can choose to use other formats like protobuf, JSON, XML,
 * etc.
 */
esp_err_t Wifi::custom_prov_data_handler(uint32_t session_id,
                                         const uint8_t* inbuf, ssize_t inlen,
                                         uint8_t** outbuf, ssize_t* outlen,
                                         void* priv_data) {
  if (inbuf) {
    Log::info("NET::WIFI::PROV", "Received data: %.*s", inlen, (char*)inbuf);
  }
  char response[] = "SUCCESS";
  *outbuf = (uint8_t*)strdup(response);
  if (*outbuf == NULL) {
    Log::error("NET::WIFI::PROV", "System out of memory");
    return ESP_ERR_NO_MEM;
  }
  *outlen = strlen(response) + 1; /* +1 for NULL terminating byte */

  return ESP_OK;
}

void Wifi::start_ble_provisioning() {
  /* Initialize Wi-Fi including netif with default config */
  // close_if();
  // interface = esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  /* Configuration for the provisioning manager */
  wifi_prov_mgr_config_t config = {
      .scheme = wifi_prov_scheme_ble,
      .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};

  /* Initialize provisioning manager with the
   * configuration parameters set above */
  ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

  // bool provisioned = false;
  /* Let's find out if the device is provisioned */
  // ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

  /* If device is not yet provisioned start provisioning service */
  // if (!provisioned) {
  Log::info("NET::WIFI::PROV", "Starting provisioning");

  /* What is the Device Service Name that we want
   * This translates to :
   *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
   *     - device name when scheme is wifi_prov_scheme_ble
   */
  char service_name[12];
  get_device_service_name(service_name, sizeof(service_name));

  /* What is the security level that we want (0 or 1):
   *      - WIFI_PROV_SECURITY_0 is simply plain text communication.
   *      - WIFI_PROV_SECURITY_1 is secure communication which consists of
   * secure handshake using X25519 key exchange and proof of possession (pop)
   * and AES-CTR for encryption/decryption of messages.
   */
  wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

  /* Do we want a proof-of-possession (ignored if Security 0 is selected):
   *      - this should be a string with length > 0
   *      - NULL if not used
   */
  const char* pop = NULL;

  /* What is the service key (could be NULL)
   * This translates to :
   *     - Wi-Fi password when scheme is wifi_prov_scheme_softap
   *     - simply ignored when scheme is wifi_prov_scheme_ble
   */
  const char* service_key = NULL;

  /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
   * set a custom 128 bit UUID which will be included in the BLE advertisement
   * and will correspond to the primary GATT service that provides
   * provisioning endpoints as GATT characteristics. Each GATT characteristic
   * will be formed using the primary service UUID as base, with different
   * auto assigned 12th and 13th bytes (assume counting starts from 0th byte).
   * The client side applications must identify the endpoints by reading the
   * User Characteristic Description descriptor (0x2901) for each
   * characteristic, which contains the endpoint name of the characteristic */
  uint8_t custom_service_uuid[] = {
      /* LSB <---------------------------------------
       * ---------------------------------------> MSB */
      0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
      0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
  };
  wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

  /* An optional endpoint that applications can create if they expect to
   * get some additional custom data during provisioning workflow.
   * The endpoint name can be anything of your choice.
   * This call must be made before starting the provisioning.
   */
  wifi_prov_mgr_endpoint_create("custom-data");
  /* Start provisioning service */
  ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop, service_name,
                                                   service_key));

  provisioning_active = true;

  /* The handler for the optional endpoint created above.
   * This call must be made after starting the provisioning, and only if the
   * endpoint has already been created above.
   */
  wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler,
                                  nullptr);
}

void Wifi::end_ble_provisioning() {
  wifi_prov_mgr_stop_provisioning();
  provisioning_active = false;
  should_save_creds = false;
  prov_success = false;
}

void Wifi::get_device_service_name(char* service_name, size_t max) {
  uint8_t eth_mac[6];
  const char* ssid_prefix = "PROV_";
  esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
  snprintf(service_name, max, "%s%02X%02X%02X", ssid_prefix, eth_mac[3],
           eth_mac[4], eth_mac[5]);
}

void Wifi::close_if() {
  if (interface) {
    esp_netif_destroy(interface);
    interface = nullptr;
  }
}

std::string Wifi::get_mac_address() {
  std::stringstream mac;

  std::array<uint8_t, 6> m;
  bool ret = get_local_mac_address(m);

  if (ret) {
    for (const auto& v : m) {
      if (mac.tellp() > 0) {
        mac << "_";
      }

      mac << std::hex << static_cast<int>(v);
    }
  }

  return mac.str();
}

bool Wifi::get_local_mac_address(std::array<uint8_t, 6>& m) {
  wifi_mode_t mode;
  esp_err_t err = esp_wifi_get_mode(&mode);

  if (err == ESP_OK) {
    if (mode == WIFI_MODE_STA) {
      err = esp_wifi_get_mac(WIFI_IF_STA, m.data());
    } else if (mode == WIFI_MODE_AP) {
      err = esp_wifi_get_mac(WIFI_IF_AP, m.data());
    } else {
      err = ESP_FAIL;
    }
  }

  if (err != ESP_OK) {
    Log::error("Wifi", "get_local_mac_address(): {}", esp_err_to_name(err));
  }

  return err == ESP_OK;
}

// attention: access to this function might have a threading issue.
// It should be called from the main thread only!
uint32_t Wifi::get_local_ip() { return ip.addr; }

void Wifi::start_softap(uint8_t max_conn) {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  wifi_config_t config{};

  copy_min_to_buffer(ssid.begin(), ssid.length(), config.ap.ssid);
  copy_min_to_buffer(password.begin(), password.length(), config.ap.password);

  config.ap.max_connection = max_conn;
  config.ap.authmode =
      password.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_WPA2_PSK;

  close_if();
  interface = esp_netif_create_default_wifi_ap();

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(ESP_IF_WIFI_AP, &config);
  esp_wifi_start();

  Log::info("SoftAP", "SSID: {}; Auth {}", ssid,
            (password.empty() ? "Open" : "WPA2/PSK"));

#ifndef ESP_PLATFORM

  // Assume network is available when running under POSIX system.
  publish_status(true, true);
#endif
}

void Wifi::publish_status(bool connected, bool ip_changed) {
  network::NetworkStatus status(connected ? network::NetworkEvent::GOT_IP
                                          : network::NetworkEvent::DISCONNECTED,
                                ip_changed);
  core::ipc::Publisher<network::NetworkStatus>::publish(status);
}

void Wifi::connect() const {
#ifdef ESP_PLATFORM
  esp_wifi_start();
  esp_wifi_connect();
#else
  // Assume network is available when running under POSIX system.
  publish_status(true, true);
#endif
}
}  // namespace smooth::core::network
