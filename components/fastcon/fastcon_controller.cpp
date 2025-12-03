#include "esphome/core/component_iterator.h"
#include "esphome/core/log.h"
#include "esphome/components/light/color_mode.h"
#include "fastcon_controller.h"
#include "protocol.h"

namespace esphome
{
    namespace fastcon
    {
        static const char *const TAG = "fastcon.controller";

        void FastconController::queueCommand(uint32_t light_id_, const std::vector<uint8_t> &data)
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (queue_.size() >= max_queue_size_)
            {
                ESP_LOGW(TAG, "Command queue full (size=%d), dropping command for light %d",
                         queue_.size(), light_id_);
                return;
            }

            Command cmd;
            cmd.data = data;
            cmd.timestamp = millis();
            cmd.retries = 0;

            queue_.push(cmd);
            ESP_LOGV(TAG, "Command queued, queue size: %d", queue_.size());
        }

        void FastconController::clear_queue()
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            std::queue<Command> empty;
            std::swap(queue_, empty);
        }

        void FastconController::setup()
        {
            ESP_LOGCONFIG(TAG, "Setting up Fastcon BLE Controller...");
            ESP_LOGCONFIG(TAG, "  Advertisement interval: %d-%d", this->adv_interval_min_, this->adv_interval_max_);
            ESP_LOGCONFIG(TAG, "  Advertisement duration: %dms", this->adv_duration_);
            ESP_LOGCONFIG(TAG, "  Advertisement gap: %dms", this->adv_gap_);
        }

        void FastconController::loop()
        {
            const uint32_t now = millis();

            // Handle pairing mode - this takes priority over normal operations
            if (pairing_mode_)
            {
                uint32_t elapsed = now - pairing_start_time_;
                
                // Phase transition: Discovery (4s) -> Pairing (continues until timeout)
                if (pairing_phase_ == PairingPhase::DISCOVERY && elapsed >= 4000)
                {
                    ESP_LOGI(TAG, "Discovery phase complete - switching to PAIRING phase");
                    ESP_LOGI(TAG, "Will now broadcast pairing packets with Light ID %d", pairing_light_id_);
                    pairing_phase_ = PairingPhase::PAIRING;
                    pairing_phase_start_ = now;  // Track when we entered pairing phase
                    // Force new advertisement by resetting state
                    esp_ble_gap_stop_advertising();
                    adv_state_ = AdvertiseState::IDLE;
                }
                
                // Auto-increment Light ID every 5 seconds during pairing phase
                if (pairing_phase_ == PairingPhase::PAIRING)
                {
                    uint32_t phase_elapsed = now - pairing_phase_start_;
                    uint32_t current_light_slot = phase_elapsed / 5000;
                    uint32_t new_light_id = pairing_base_light_id_ + current_light_slot;  // Calculate from base ID
                    
                    // Check if we've moved to a new Light ID slot
                    if (new_light_id != pairing_light_id_)
                    {
                        pairing_light_id_ = new_light_id;
                        ESP_LOGI(TAG, "Auto-incrementing to Light ID %d", pairing_light_id_);
                        sequence_counter_ = 0x50;  // Reset sequence for new Light ID
                        esp_ble_gap_stop_advertising();
                        adv_state_ = AdvertiseState::IDLE;
                    }
                }
                
                // Exit pairing mode after 60 seconds total
                if (elapsed >= 60000)
                {
                    ESP_LOGI(TAG, "Pairing timeout (60s) - exiting pairing mode");
                    pairing_mode_ = false;
                    esp_ble_gap_stop_advertising();
                    adv_state_ = AdvertiseState::IDLE;
                    
                    // Restart BLE scanning
                    ESP_LOGI(TAG, "Restarting BLE scanner");
                    esp_ble_gap_start_scanning(300);  // Resume scanning
                    return;
                }
                
                // Continuously advertise during pairing mode
                if (adv_state_ == AdvertiseState::IDLE || 
                    (adv_state_ == AdvertiseState::ADVERTISING && (now - state_start_time_ >= 100)))
                {
                    // Stop previous advertisement if any
                    if (adv_state_ == AdvertiseState::ADVERTISING)
                    {
                        esp_ble_gap_stop_advertising();
                    }
                    
                    // Build the appropriate advertisement based on phase
                    std::vector<uint8_t> adv_data;
                    if (pairing_phase_ == PairingPhase::DISCOVERY)
                    {
                        adv_data = build_discovery_advertisement();
                        ESP_LOGD(TAG, "Broadcasting discovery advertisement (0x4e)");
                    }
                    else
                    {
                        adv_data = build_pairing_advertisement();
                        ESP_LOGD(TAG, "Broadcasting pairing advertisement (0x6e) with Light ID %d", pairing_light_id_);
                    }
                    
                    // Configure BLE advertisement parameters for rapid pairing broadcasts
                    esp_ble_adv_params_t adv_params = {
                        .adv_int_min = 0x20,  // 20ms minimum
                        .adv_int_max = 0x40,  // 40ms maximum - very fast for pairing
                        .adv_type = ADV_TYPE_NONCONN_IND,
                        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                        .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
                        .channel_map = ADV_CHNL_ALL,
                        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
                    };
                    
                    // Set the raw advertisement data (skip MAC bytes, start from AD flags)
                    uint8_t adv_data_raw[31] = {0};
                    size_t copy_len = std::min(adv_data.size() - 6, (size_t)25);  // Skip 6-byte MAC, limit to 25
                    memcpy(adv_data_raw, adv_data.data() + 6, copy_len);
                    
                    esp_err_t err = esp_ble_gap_config_adv_data_raw(adv_data_raw, copy_len);
                    if (err != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Error setting pairing advertisement data: %s", esp_err_to_name(err));
                        return;
                    }
                    
                    err = esp_ble_gap_start_advertising(&adv_params);
                    if (err != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Error starting pairing advertisement: %s", esp_err_to_name(err));
                        return;
                    }
                    
                    adv_state_ = AdvertiseState::ADVERTISING;
                    state_start_time_ = now;
                }
                
                // Keep advertising during pairing mode
                return;
            }

            // Normal operation - handle advertisement state machine
            switch (adv_state_)
            {
            case AdvertiseState::IDLE:
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (queue_.empty())
                    return;

                Command cmd = queue_.front();
                queue_.pop();

                esp_ble_adv_params_t adv_params = {
                    .adv_int_min = adv_interval_min_,
                    .adv_int_max = adv_interval_max_,
                    .adv_type = ADV_TYPE_NONCONN_IND,
                    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
                    .channel_map = ADV_CHNL_ALL,
                    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
                };

                uint8_t adv_data_raw[31] = {0};
                uint8_t adv_data_len = 0;

                // Add flags
                adv_data_raw[adv_data_len++] = 2;
                adv_data_raw[adv_data_len++] = ESP_BLE_AD_TYPE_FLAG;
                adv_data_raw[adv_data_len++] = ESP_BLE_ADV_FLAG_BREDR_NOT_SPT | ESP_BLE_ADV_FLAG_GEN_DISC;

                // Add manufacturer data
                adv_data_raw[adv_data_len++] = cmd.data.size() + 2;
                adv_data_raw[adv_data_len++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
                adv_data_raw[adv_data_len++] = MANUFACTURER_DATA_ID & 0xFF;
                adv_data_raw[adv_data_len++] = (MANUFACTURER_DATA_ID >> 8) & 0xFF;

                memcpy(&adv_data_raw[adv_data_len], cmd.data.data(), cmd.data.size());
                adv_data_len += cmd.data.size();

                esp_err_t err = esp_ble_gap_config_adv_data_raw(adv_data_raw, adv_data_len);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "Error setting raw advertisement data (err=%d): %s", err, esp_err_to_name(err));
                    return;
                }

                err = esp_ble_gap_start_advertising(&adv_params);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "Error starting advertisement (err=%d): %s", err, esp_err_to_name(err));
                    return;
                }

                adv_state_ = AdvertiseState::ADVERTISING;
                state_start_time_ = now;
                ESP_LOGV(TAG, "Started advertising");
                break;
            }

            case AdvertiseState::ADVERTISING:
            {
                if (now - state_start_time_ >= adv_duration_)
                {
                    esp_ble_gap_stop_advertising();
                    adv_state_ = AdvertiseState::GAP;
                    state_start_time_ = now;
                    ESP_LOGV(TAG, "Stopped advertising, entering gap period");
                }
                break;
            }

            case AdvertiseState::GAP:
            {
                if (now - state_start_time_ >= adv_gap_)
                {
                    adv_state_ = AdvertiseState::IDLE;
                    ESP_LOGV(TAG, "Gap period complete");
                }
                break;
            }
            }
        }

        std::vector<uint8_t> FastconController::get_light_data(light::LightState *state)
        {
            std::vector<uint8_t> light_data = {
                0, // 0 - On/Off Bit + 7-bit Brightness
                0, // 1 - Blue byte
                0, // 2 - Red byte
                0, // 3 - Green byte
                0, // 4 - Warm byte
                0  // 5 - Cold byte
            };

            // TODO: need to figure out when esphome is changing to white vs setting brightness

            auto values = state->current_values;

            bool is_on = values.is_on();
            if (!is_on)
            {
                return std::vector<uint8_t>({0x00});
            }

            auto color_mode = values.get_color_mode();
            bool has_white = (static_cast<uint8_t>(color_mode) & static_cast<uint8_t>(light::ColorCapability::WHITE)) != 0;
            float brightness = std::min(values.get_brightness() * 127.0f, 127.0f); // clamp the value to at most 127
            light_data[0] = 0x80 + static_cast<uint8_t>(brightness);

            if (has_white)
            {
                return std::vector<uint8_t>({static_cast<uint8_t>(brightness)});
                // DEBUG: when changing to white mode, this should be the payload:
                // ff0000007f7f
            }

            bool has_rgb = (static_cast<uint8_t>(color_mode) & static_cast<uint8_t>(light::ColorCapability::RGB)) != 0;
            if (has_rgb)
            {
                light_data[1] = static_cast<uint8_t>(values.get_blue() * 255.0f);
                light_data[2] = static_cast<uint8_t>(values.get_red() * 255.0f);
                light_data[3] = static_cast<uint8_t>(values.get_green() * 255.0f);
            }

            bool has_cold_warm = (static_cast<uint8_t>(color_mode) & static_cast<uint8_t>(light::ColorCapability::COLD_WARM_WHITE)) != 0;
            if (has_cold_warm)
            {
                light_data[4] = static_cast<uint8_t>(values.get_warm_white() * 255.0f);
                light_data[5] = static_cast<uint8_t>(values.get_cold_white() * 255.0f);
            }

            // TODO figure out if we can use these, and how
            bool has_temp = (static_cast<uint8_t>(color_mode) & static_cast<uint8_t>(light::ColorCapability::COLOR_TEMPERATURE)) != 0;
            if (has_temp)
            {
                float temperature = values.get_color_temperature();
                if (temperature < 153)
                {
                    light_data[4] = 0xff;
                    light_data[5] = 0x00;
                }
                else if (temperature > 500)
                {
                    light_data[4] = 0x00;
                    light_data[5] = 0xff;
                }
                else
                {
                    // Linear interpolation between (153, 0xff) and (500, 0x00)
                    light_data[4] = (uint8_t)(((500 - temperature) * 255.0f + (temperature - 153) * 0x00) / (500 - 153));
                    light_data[5] = (uint8_t)(((temperature - 153) * 255.0f + (500 - temperature) * 0x00) / (500 - 153));
                }
            }

            return light_data;
        }

        std::vector<uint8_t> FastconController::single_control(uint32_t light_id_, const std::vector<uint8_t> &light_data)
        {
            std::vector<uint8_t> result_data(12);

            result_data[0] = 2 | (((0xfffffff & (light_data.size() + 1)) << 4));
            result_data[1] = light_id_;
            std::copy(light_data.begin(), light_data.end(), result_data.begin() + 2);

            // Debug output - print payload as hex
            auto hex_str = vector_to_hex_string(result_data).data();
            ESP_LOGD(TAG, "Inner Payload (%d bytes): %s", result_data.size(), hex_str);

            return this->generate_command(5, light_id_, result_data, true);
        }

        void FastconController::send_raw_command(uint32_t light_id, const std::vector<uint8_t> &data)
        {
            // Generate mesh packet with command type 5 (control)
            std::vector<uint8_t> mesh_packet = generate_command(5, light_id, data, true);
            queueCommand(light_id, mesh_packet);
        }

        std::vector<uint8_t> FastconController::generate_command(uint8_t n, uint32_t light_id_, const std::vector<uint8_t> &data, bool forward)
        {
            static uint8_t sequence = 0;

            // Create command body with header
            std::vector<uint8_t> body(data.size() + 4);
            uint8_t i2 = (light_id_ / 256);

            // Construct header
            body[0] = (i2 & 0b1111) | ((n & 0b111) << 4) | (forward ? 0x80 : 0);
            body[1] = sequence++; // Use and increment sequence number
            if (sequence >= 255)
                sequence = 1;

            body[2] = this->mesh_key_[3]; // Safe key

            // Copy data
            std::copy(data.begin(), data.end(), body.begin() + 4);

            // Calculate checksum
            uint8_t checksum = 0;
            for (size_t i = 0; i < body.size(); i++)
            {
                if (i != 3)
                {
                    checksum = checksum + body[i];
                }
            }
            body[3] = checksum;

            // Encrypt header and data
            for (size_t i = 0; i < 4; i++)
            {
                body[i] = DEFAULT_ENCRYPT_KEY[i & 3] ^ body[i];
            }

            for (size_t i = 0; i < data.size(); i++)
            {
                body[4 + i] = this->mesh_key_[i & 3] ^ body[4 + i];
            }

            // Prepare the final payload with RF protocol formatting
            std::vector<uint8_t> addr = {DEFAULT_BLE_FASTCON_ADDRESS.begin(), DEFAULT_BLE_FASTCON_ADDRESS.end()};
            return prepare_payload(addr, body);
        }

        void FastconController::pair_device(uint32_t new_light_id, uint32_t group_id)
        {
            // NEW PAIRING PROTOCOL DISCOVERED FROM BLE CAPTURE:
            // The phone/controller advertises with fake MAC 11:22:33:44:55:66
            // and sends the mesh key in a raw BLE advertisement packet.
            // This is NOT a mesh command - it's a pure BLE advertisement!
            //
            // Two-phase process:
            // 1. Discovery phase (~4 seconds): Command byte 0x4e
            // 2. Pairing phase (~0.3 seconds): Command byte 0x6e with mesh key
            
            ESP_LOGI(TAG, "=== Starting BLE Pairing Mode for Light ID %d (Group %d) ===", new_light_id, group_id);
            ESP_LOGI(TAG, "This will broadcast pairing advertisements for 60 seconds");
            ESP_LOGI(TAG, "Make sure your light is in factory reset / pairing mode!");
            
            // CRITICAL: Stop BLE scanning - it blocks advertisements!
            ESP_LOGI(TAG, "Stopping BLE scanner to enable pairing advertisements");
            esp_ble_gap_stop_scanning();
            
            // Stop any existing advertisements and reset state
            esp_ble_gap_stop_advertising();
            
            // Store pairing state
            pairing_mode_ = true;
            pairing_start_time_ = millis();
            pairing_light_id_ = new_light_id;
            pairing_base_light_id_ = new_light_id;  // Remember starting ID
            pairing_phase_ = PairingPhase::DISCOVERY;
            adv_state_ = AdvertiseState::IDLE;  // CRITICAL: Initialize state for pairing
            
            // We'll handle the actual advertisement in loop()
            ESP_LOGI(TAG, "Pairing mode activated - entering DISCOVERY phase");
        }

        void FastconController::factory_reset_device(uint32_t light_id)
        {
            ESP_LOGI(TAG, "Sending factory reset to Light ID %d", light_id);
            
            // Factory reset command: all zeros payload
            std::vector<uint8_t> reset_data(7, 0x00);
            
            // Send reset command
            std::vector<uint8_t> mesh_packet = generate_command(5, light_id, reset_data, true);
            queueCommand(light_id, mesh_packet);
            
            ESP_LOGI(TAG, "Factory reset command queued");
        }

        std::vector<uint8_t> FastconController::build_discovery_advertisement()
        {
            // Discovery phase advertisement (command 0x4e)
            // Example from capture: 66554433221102011a13fff0ff4e6c5a05348e89b5e238a1a85e367bc4e9974d
            
            std::vector<uint8_t> adv_data;
            
            // MAC Address (reversed): 11:22:33:44:55:66 -> 66 55 44 33 22 11
            adv_data.insert(adv_data.end(), {0x66, 0x55, 0x44, 0x33, 0x22, 0x11});
            
            // AD Flags structure
            adv_data.insert(adv_data.end(), {0x02, 0x01, 0x1a});
            
            // Manufacturer Specific Data structure
            adv_data.push_back(0x13);  // Length: 19 bytes
            adv_data.push_back(0xff);  // Type: Manufacturer Specific
            adv_data.push_back(0xf0);  // Company ID: 0xf0ff (little-endian)
            adv_data.push_back(0xff);
            
            // Command byte for discovery
            adv_data.push_back(0x4e);  // 'N' - discovery mode
            
            // Variable data (7 bytes) - using random values for now
            // These appear to change between packets in the capture
            static uint8_t counter = 0;
            adv_data.insert(adv_data.end(), {
                static_cast<uint8_t>(0x6c + (counter % 4)),
                static_cast<uint8_t>(0x5a + (counter % 8)),
                static_cast<uint8_t>(counter % 8),
                0x34, 0x8e, 0x89, 0xb5
            });
            counter++;
            
            // Placeholder for remaining bytes (we'll improve this later)
            adv_data.insert(adv_data.end(), {0xe2, 0x38, 0xa1, 0xa8, 0x5e, 0x36, 0x7b, 0xc4});
            
            // CRC (3 bytes) - placeholder for now
            adv_data.insert(adv_data.end(), {0xe9, 0x97, 0x4d});
            
            return adv_data;
        }

        std::vector<uint8_t> FastconController::build_pairing_advertisement()
        {
            // Pairing phase advertisement (command 0x6e) with mesh key
            // Example: 66554433221102011a13fff0ff6e50596344103332340a3939303233367cb212
            // Decoded: nPYcD.324.990236|..
            
            std::vector<uint8_t> adv_data;
            
            // MAC Address (reversed): 11:22:33:44:55:66 -> 66 55 44 33 22 11
            adv_data.insert(adv_data.end(), {0x66, 0x55, 0x44, 0x33, 0x22, 0x11});
            
            // AD Flags structure
            adv_data.insert(adv_data.end(), {0x02, 0x01, 0x1a});
            
            // Manufacturer Specific Data structure
            adv_data.push_back(0x13);  // Length: 19 bytes
            adv_data.push_back(0xff);  // Type: Manufacturer Specific
            adv_data.push_back(0xf0);  // Company ID: 0xf0ff (little-endian)
            adv_data.push_back(0xff);
            
            // Command byte for pairing
            adv_data.push_back(0x6e);  // 'n' - pairing mode
            
            // Sequence counter (increments with each packet)
            static uint8_t pairing_counter = 0x50;
            adv_data.push_back(pairing_counter++);
            
            // CRITICAL: Light ID assignment (16-bit little-endian at bytes 2-3)
            // Based on brmesh-pairing.yaml: uint16_t light_id = (mfg_data[7] << 8) | mfg_data[6]
            // In full packet that's offsets 13+6=19 and 13+7=20, so mfg_data[6-7]
            // Here in mfg_data it's positions 2-3 after command+counter
            ESP_LOGD(TAG, "Including Light ID %d (0x%04x) in pairing packet", pairing_light_id_, pairing_light_id_);
            adv_data.push_back(static_cast<uint8_t>(pairing_light_id_ & 0xFF));        // Low byte
            adv_data.push_back(static_cast<uint8_t>((pairing_light_id_ >> 8) & 0xFF)); // High byte
            
            // Variable data (4 bytes) - pattern observed in captures
            adv_data.insert(adv_data.end(), {
                0x44,
                0x10,
                0x33,
                0x32
            });
            
            // Sequence: "34\n" (0x34, 0x0a)
            adv_data.insert(adv_data.end(), {0x34, 0x0a});
            
            // Mesh key in ASCII format: "99" + hex_as_ascii
            // For mesh key 0x30323336 ("0236"), send "990236"
            adv_data.insert(adv_data.end(), {'9', '9'});  // Prefix
            
            // Convert mesh key bytes to ASCII hex representation
            for (int i = 0; i < 4; i++) {
                adv_data.push_back(mesh_key_[i]);  // Already ASCII: 0x30='0', 0x32='2', 0x33='3', 0x36='6'
            }
            
            // CRC (3 bytes) - calculate based on payload
            uint32_t crc = calculate_pairing_crc(adv_data);
            adv_data.push_back((crc >> 16) & 0xFF);
            adv_data.push_back((crc >> 8) & 0xFF);
            adv_data.push_back(crc & 0xFF);
            
            std::vector<uint8_t> payload_subset(adv_data.begin() + 9, adv_data.end());
            std::vector<char> hex_chars = vector_to_hex_string(payload_subset);
            ESP_LOGD(TAG, "Pairing advertisement payload: %s", hex_chars.data());
            
            return adv_data;
        }

        uint32_t FastconController::calculate_pairing_crc(const std::vector<uint8_t> &data)
        {
            // CRC calculation - this is a placeholder
            // We need to reverse-engineer the actual algorithm from the captures
            // For now, return a simple checksum-based value
            
            uint32_t sum = 0;
            for (size_t i = 13; i < data.size(); i++) {  // Start after manufacturer header
                sum += data[i];
            }
            
            // Simple transformation to get 3 bytes
            uint32_t crc = (sum * 0x1234) & 0xFFFFFF;
            
            // TODO: Analyze multiple captures to determine the real CRC algorithm
            ESP_LOGV(TAG, "CRC calculated: 0x%06X (placeholder algorithm)", crc);
            
            return crc;
        }
    } // namespace fastcon
} // namespace esphome
