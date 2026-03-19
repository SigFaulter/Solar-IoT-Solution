#pragma once
// mqtt_client.h - Paho MQTT C++ async wrapper

#include "mqtt/async_client.h"
#include <memory>
#include <string>

struct MqttConfig {
    std::string host        = "localhost";
    int         port        = 1883;
    std::string client_id;
    std::string username;
    std::string password;
    int         keepalive_s = 60;

    // Last Will Testament - broker fires this on ungraceful disconnect
    // (power loss, crash, WAN outage).  Leave lwt_topic empty to disable.
    std::string lwt_topic;
    std::string lwt_payload = "0";
    bool        lwt_retain  = true;

    int publish_timeout_ms = 5000;
};


class MqttClient {
public:
    explicit MqttClient(const MqttConfig& cfg);
    ~MqttClient();

    bool connect();
    void disconnect();
    bool connected() const { return connected_; }

    /**
     * Publish a message and block until acknowledged (up to publish_timeout_ms).
     *
     * @param topic   MQTT topic string
     * @param payload UTF-8 payload
     * @param retain  whether the broker should retain this message
     * @param qos     0 = fire-and-forget, 1 = at-least-once (default 0)
     */
    bool publish(const std::string& topic,
                 const std::string& payload,
                 bool               retain = false,
                 int                qos    = 0);

private:
    MqttConfig                          cfg_;
    std::unique_ptr<mqtt::async_client> client_;
    bool                                connected_ = false;
};
