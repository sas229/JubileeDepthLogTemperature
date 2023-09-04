#include <stdio.h>
#include <string>
#include <iostream>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "json.hpp"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "one_wire.h"

// UDP settings.
const char hostname[] = "DLT";
char server_ip[] = "192.168.0.168";
const int signalk_server_port = 4123;
const int buffer_size = 300;
const char ssid[] = "OoohAaarGetOffMyWAN";
const char pass[] = "a1b2c3d4";

// Other defines.
#define PONG_GPIO_PIN 10
#define PING_GPIO_PIN 11
#define PULSE_GPIO_PIN 16
#define TEMP_SENSE_GPIO_PIN 17

// Namespaces.
using namespace std;
using json = nlohmann::json;

// Data objects.
struct udp_pcb* upcb;
struct udp_pcb* spcb;
char buffer[buffer_size];
int pulses = 0;
absolute_time_t echo_send;
absolute_time_t echo_arrival;
absolute_time_t pulse_start;
absolute_time_t pulse_end;
bool measuring_depth = false;
double depth = 0.0;
double speed = 0.0;
double water_temperature = 0.0;
int value_index = 0;

void send_udp(char* IP , int port, void* data, int data_size) {
    ip_addr_t destAddr;
    ip4addr_aton(IP, &destAddr);
    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, data_size, PBUF_RAM);
    memcpy(p->payload, data, data_size);
    cyw43_arch_lwip_begin();
    udp_sendto(upcb, p, &destAddr, port);
    cyw43_arch_lwip_end();
    pbuf_free(p);
}

int check_wifi_signal() {
    int32_t rssi;
    cyw43_ioctl(&cyw43_state, 254, sizeof(rssi), (uint8_t*)&rssi, CYW43_ITF_STA);
    if (rssi >= -67) {
        return 5;
    } else if (rssi > -67 && rssi <= -70) {
        return 4;
    } else if (rssi > -70 && rssi <= -80) {
        return 3;
    } else if (rssi > -80 && rssi <= -90) {
        return 2;
    } else {
        return 1;
    }
}

void connect_to_wifi(const char ssid[], const char pass[]) {
    bool connected = false;
    while (!connected) {
        int ret = cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000);
        if (ret == 0) {
            connected = true;
        } else {
            printf("WiFi failed to connect to network. Retrying...\n");
        }
    }
}

void pulse_received(uint gpio, uint32_t events) {
    // Increment pulse count.
    pulses += 1;
}

void pong_received(uint gpio, uint32_t events) {
    // Log arrival time.
    echo_arrival = get_absolute_time();
    measuring_depth = false;
}

int main() {    
    stdio_init_all();
    sleep_ms(2000);

    // WiFi setup.
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
        printf("Failed to initialise...\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    
    // Set custom device name.
    cyw43_arch_lwip_begin();
    struct netif *n = &cyw43_state.netif[CYW43_ITF_STA];
    netif_set_hostname(n, hostname);
    netif_set_up(n);
    cyw43_arch_lwip_end();
    
    // Connect to WiFi.
    connect_to_wifi(ssid, pass);

    // Check signal strength.
    int signal_strength = check_wifi_signal();
    printf("Connected to %s with signal strength: %i\n", ssid, signal_strength);

    // Illuminate LED if connection is successful.
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Setup UDP interface.
    upcb = udp_new();   
    spcb = udp_new();
    err_t err = udp_bind(spcb, IP_ADDR_ANY, signalk_server_port);

    // Setup one wire interfaace.
    One_wire one_wire(TEMP_SENSE_GPIO_PIN);
	one_wire.init();
	sleep_ms(1);

    // Pulse pin.
    gpio_init(PULSE_GPIO_PIN);
    gpio_set_dir(PULSE_GPIO_PIN, GPIO_IN);
    gpio_pull_down(PULSE_GPIO_PIN);
    gpio_set_irq_enabled_with_callback(PULSE_GPIO_PIN, GPIO_IRQ_EDGE_FALL, true, &pulse_received);
    
    // Send ping pin.
    gpio_init(PING_GPIO_PIN);
    gpio_set_dir(PING_GPIO_PIN, GPIO_OUT);

    // Receive pong pin.
    gpio_init(PONG_GPIO_PIN);
    gpio_set_dir(PONG_GPIO_PIN, GPIO_IN);
    gpio_pull_up(PONG_GPIO_PIN);
    gpio_set_irq_enabled_with_callback(PONG_GPIO_PIN, GPIO_IRQ_EDGE_FALL, true, &pong_received);

    // Setup json output structure.
    json out;
    out["updates"][0]["source"] = {{"label", "JubileeDepthLogTemperature"}, {"type", "SignalK"}, {"src","dlt"}};

    while (true) {
        // Check WiFi status.
        int ret = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (ret != 3) {
            printf("WiFi disconnected. Reconnecting...\n");
            connect_to_wifi(ssid, pass);
        }
        signal_strength = check_wifi_signal();
        out["updates"][0]["values"][value_index] = {{"path", "communication.JubileeDepthLogTemperature.WiFiSignalStrength"}, {"value", signal_strength}};
        value_index += 1;

        // Send echo sounder ping.
        gpio_put(PING_GPIO_PIN, 1);
        echo_send = get_absolute_time();
        measuring_depth = true;

        // Log temperature.
        rom_address_t address{};
        one_wire.single_device_read_rom(address);
        if (One_wire::to_uint64(address) == 0){
            printf("No temperature sensor found.\n");
        } else {
            printf("Temperature sensor address: %i\n", One_wire::to_uint64(address));
            one_wire.convert_temperature(address, true, false);
            water_temperature = one_wire.temperature(address);
            out["updates"][0]["values"][1] = {{"path", "environment.water.temperature"}, {"value", water_temperature}};
            value_index += 1;
        }
        
        // Convert pulses to speed.
        pulse_end = get_absolute_time();
        double elapsed = (double)absolute_time_diff_us(pulse_start, pulse_end);
        double elapsed_per_pulse = elapsed/(double)pulses;
        speed = 0.278*1000.0/elapsed_per_pulse;
        out["updates"][0]["values"][value_index] = {{"path", "navigation.speedThroughWater"}, {"value", speed}};
        value_index += 1;
        pulse_start = get_absolute_time();
        pulses = 0;

        // Check for echo sounder pong.
        if (measuring_depth == false) {
            double elapsed = (double)absolute_time_diff_us(echo_send, echo_arrival);
            depth = (1493.0/(1000.0*1000.0))*elapsed/2.0;
        }
        out["updates"][0]["values"][value_index] = {{"path", "environment.depth.belowTransducer"}, {"value", depth}};
        value_index = 0;

        // Send to SignalK server.
        std::cout << out.dump() << std::endl;
        std::string output_str = out.dump();
        std::vector<char> v(output_str.begin(), output_str.end());
        void* p = &v[0];
        send_udp(server_ip, signalk_server_port, p, v.size());
        
        // Repeat every second.
        sleep_ms(1000);
    }

    return 0;
}