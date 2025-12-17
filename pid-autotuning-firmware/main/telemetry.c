#include "telemetry.h"

static const char *TAG = "TELEMETRY";

#define PROTOCOL_VERSION 1

// Internal state
typedef struct {
    int socket_fd;
    char server_ip[16];
    telemetry_status_t status;
    
    // Data buffers - use union to save memory
    union {
        data_batch_t data_batch;           // For regular telemetry
        ident_data_batch_t ident_batch;    // For identification
    };
    
    uint16_t current_sample_idx;
    uint16_t sequence_number;
    
    // Statistics
    uint32_t samples_sent;
    uint32_t responses_received;
    uint32_t errors;
    
    // Thread safety
    SemaphoreHandle_t mutex;
} telemetry_state_t;

static telemetry_state_t g_telem = {
    .socket_fd = -1,
    .status = TELEMETRY_IDLE,
    .current_sample_idx = 0,
    .sequence_number = 0,
    .samples_sent = 0,
    .responses_received = 0,
    .errors = 0
};

// CRC32 calculation (simple implementation)
static uint32_t calculate_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// Initialize telemetry system
esp_err_t telemetry_init(const char *server_ip) {
    if (g_telem.mutex == NULL) {
        g_telem.mutex = xSemaphoreCreateMutex();
        if (g_telem.mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_FAIL;
        }
    }
    
    strncpy(g_telem.server_ip, server_ip, sizeof(g_telem.server_ip) - 1);
    g_telem.status = TELEMETRY_IDLE;
    
    ESP_LOGI(TAG, "Telemetry initialized for server: %s", server_ip);
    return ESP_OK;
}

// Connect to server
esp_err_t telemetry_connect(void) {
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd >= 0) {
        ESP_LOGW(TAG, "Already connected");
        xSemaphoreGive(g_telem.mutex);
        return ESP_OK;
    }
    
    // Create socket
    g_telem.socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_telem.socket_fd < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Set socket timeout
    struct timeval timeout = {
        .tv_sec = TELEMETRY_TIMEOUT_MS / 1000,
        .tv_usec = (TELEMETRY_TIMEOUT_MS % 1000) * 1000
    };
    setsockopt(g_telem.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(g_telem.socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Configure server address
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TELEMETRY_SERVER_PORT)
    };
    inet_pton(AF_INET, g_telem.server_ip, &server_addr.sin_addr);
    
    // Connect
    ESP_LOGI(TAG, "Connecting to %s:%d", g_telem.server_ip, TELEMETRY_SERVER_PORT);
    if (connect(g_telem.socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Connection failed: errno %d", errno);
        close(g_telem.socket_fd);
        g_telem.socket_fd = -1;
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    g_telem.status = TELEMETRY_CONNECTED;
    ESP_LOGI(TAG, "Connected successfully");
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

esp_err_t telemetry_ident_connect(void)
{
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd >= 0) {
        ESP_LOGW(TAG, "Already connected");
        xSemaphoreGive(g_telem.mutex);
        return ESP_OK;
    }
    
    // Create socket
    g_telem.socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_telem.socket_fd < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Set socket timeout
    struct timeval timeout = {
        .tv_sec = TELEMETRY_TIMEOUT_MS / 1000,
        .tv_usec = (TELEMETRY_TIMEOUT_MS % 1000) * 1000
    };
    setsockopt(g_telem.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(g_telem.socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Configure server address
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(IDENT_SERVER_PORT)
    };
    inet_pton(AF_INET, g_telem.server_ip, &server_addr.sin_addr);
    
    // Connect
    ESP_LOGI(TAG, "Connecting to %s:%d", g_telem.server_ip, IDENT_SERVER_PORT);
    if (connect(g_telem.socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Connection failed: errno %d", errno);
        close(g_telem.socket_fd);
        g_telem.socket_fd = -1;
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    g_telem.status = TELEMETRY_CONNECTED;
    ESP_LOGI(TAG, "Connected successfully to identification server");
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

// Disconnect from server
esp_err_t telemetry_disconnect(void) {
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd >= 0) {
        close(g_telem.socket_fd);
        g_telem.socket_fd = -1;
    }
    
    g_telem.status = TELEMETRY_IDLE;
    ESP_LOGI(TAG, "Disconnected");
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

// Add a sample to the batch
esp_err_t telemetry_add_sample(const robot_sample_t *sample) {
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.current_sample_idx >= TELEMETRY_SAMPLE_WINDOW) {
        ESP_LOGW(TAG, "Sample buffer full, send batch first");
        xSemaphoreGive(g_telem.mutex);
        return ESP_ERR_NO_MEM;
    }
    
    memcpy(&g_telem.data_batch.samples[g_telem.current_sample_idx], 
           sample, sizeof(robot_sample_t));
    g_telem.current_sample_idx++;
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

esp_err_t telemetry_ident_add_sample(const motor_ident_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.current_sample_idx >= TELEMETRY_SAMPLE_WINDOW) {
        ESP_LOGW(TAG, "Sample buffer full, send batch first");
        xSemaphoreGive(g_telem.mutex);
        return ESP_ERR_NO_MEM;
    }

    // Copy to ident_batch, not data_batch!
    memcpy(&g_telem.ident_batch.samples[g_telem.current_sample_idx],
           sample, sizeof(motor_ident_sample_t));
    g_telem.current_sample_idx++;
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

// Send batch to server
esp_err_t telemetry_send_batch(void) {
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd < 0) {
        ESP_LOGE(TAG, "Not connected");
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    if (g_telem.current_sample_idx == 0) {
        ESP_LOGW(TAG, "No samples to send");
        xSemaphoreGive(g_telem.mutex);
        return ESP_OK;
    }
    
    // Prepare batch
    g_telem.data_batch.sample_count = g_telem.current_sample_idx;
    g_telem.data_batch.sequence_number = g_telem.sequence_number++;
    
    // Calculate payload size
    size_t payload_size = sizeof(uint16_t) * 2 + 
                         sizeof(robot_sample_t) * g_telem.current_sample_idx;
    
    // Prepare header
    msg_header_t header = {
        .type = MSG_TYPE_DATA_BATCH,
        .version = PROTOCOL_VERSION,
        .length = payload_size
    };
    
    // Calculate checksum
    header.checksum = calculate_crc32((uint8_t *)&g_telem.data_batch, payload_size);
    
    ESP_LOGI(TAG, "Sending batch: %d samples, seq=%d", 
             g_telem.current_sample_idx, g_telem.data_batch.sequence_number);
    
    g_telem.status = TELEMETRY_SENDING;
    
    // Send header
    int sent = send(g_telem.socket_fd, &header, sizeof(header), 0);
    if (sent != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to send header: %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Send payload
    sent = send(g_telem.socket_fd, &g_telem.data_batch, payload_size, 0);
    if (sent != payload_size) {
        ESP_LOGE(TAG, "Failed to send payload: %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    g_telem.samples_sent += g_telem.current_sample_idx;
    g_telem.current_sample_idx = 0;  // Reset buffer
    g_telem.status = TELEMETRY_WAITING_RESPONSE;
    
    ESP_LOGI(TAG, "Batch sent successfully");
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

esp_err_t telemetry_ident_send_batch(void)
{
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd < 0) {
        ESP_LOGE(TAG, "Not connected");
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    if (g_telem.current_sample_idx == 0) {
        ESP_LOGW(TAG, "No samples to send");
        xSemaphoreGive(g_telem.mutex);
        return ESP_OK;
    }
    
    // Prepare ident batch
    g_telem.ident_batch.sample_count = g_telem.current_sample_idx;
    g_telem.ident_batch.sequence_number = g_telem.sequence_number++;
    
    // Calculate payload size - use ident_batch now!
    size_t payload_size = sizeof(uint16_t) * 2 + 
                         sizeof(motor_ident_sample_t) * g_telem.current_sample_idx;
    
    // Prepare header
    msg_header_t header = {
        .type = MSG_TYPE_IDENT_DATA,
        .version = PROTOCOL_VERSION,
        .length = payload_size
    };
    
    // Calculate checksum using ident_batch
    header.checksum = calculate_crc32((uint8_t *)&g_telem.ident_batch, payload_size);
    
    ESP_LOGI(TAG, "Sending identification batch: %d samples, seq=%d", 
             g_telem.current_sample_idx, g_telem.ident_batch.sequence_number);
    
    g_telem.status = TELEMETRY_SENDING;
    
    // Send header
    int sent = send(g_telem.socket_fd, &header, sizeof(header), 0);
    if (sent != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to send header: %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Send payload - use ident_batch!
    sent = send(g_telem.socket_fd, &g_telem.ident_batch, payload_size, 0);
    if (sent != payload_size) {
        ESP_LOGE(TAG, "Failed to send payload: %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    g_telem.samples_sent += g_telem.current_sample_idx;
    g_telem.current_sample_idx = 0;  // Reset buffer
    g_telem.status = TELEMETRY_WAITING_RESPONSE;

    ESP_LOGI(TAG, "Identification batch sent successfully");
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

// Request PID inference from accumulated data on server
esp_err_t telemetry_request_inference(void) {
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd < 0) {
        ESP_LOGE(TAG, "Not connected");
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Prepare header for inference request (no payload)
    msg_header_t header = {
        .type = MSG_TYPE_INFERENCE_REQUEST,
        .version = PROTOCOL_VERSION,
        .length = 0,
        .checksum = 0
    };
    
    ESP_LOGI(TAG, "Sending inference request to server...");
    
    g_telem.status = TELEMETRY_SENDING;
    
    // Send header
    int sent = send(g_telem.socket_fd, &header, sizeof(header), 0);
    if (sent != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to send inference request header: %d", errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    g_telem.status = TELEMETRY_WAITING_RESPONSE;
    ESP_LOGI(TAG, "Inference request sent successfully");
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

// Receive PID constants from server
esp_err_t telemetry_receive_pid(pid_response_t *pid_data, uint32_t timeout_ms) {
    if (pid_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (g_telem.socket_fd < 0) {
        ESP_LOGE(TAG, "Not connected");
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Set temporary timeout
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };
    setsockopt(g_telem.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // Receive header
    msg_header_t header;
    int received = recv(g_telem.socket_fd, &header, sizeof(header), 0);
    if (received != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to receive header: %d, errno: %d", received, errno);
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Validate header
    if (header.type != MSG_TYPE_PID_RESPONSE) {
        ESP_LOGE(TAG, "Unexpected message type: %d", header.type);
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    if (header.length != sizeof(pid_response_t)) {
        ESP_LOGE(TAG, "Invalid payload size: %d", header.length);
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Receive payload
    received = recv(g_telem.socket_fd, pid_data, sizeof(pid_response_t), 0);
    if (received != sizeof(pid_response_t)) {
        ESP_LOGE(TAG, "Failed to receive PID data");
        g_telem.status = TELEMETRY_ERROR;
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    // Verify checksum
    uint32_t calc_crc = calculate_crc32((uint8_t *)pid_data, sizeof(pid_response_t));
    if (calc_crc != header.checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: expected 0x%08X, got 0x%08X", 
                 header.checksum, calc_crc);
        g_telem.errors++;
        xSemaphoreGive(g_telem.mutex);
        return ESP_FAIL;
    }
    
    g_telem.responses_received++;
    g_telem.status = TELEMETRY_CONNECTED;
    
    ESP_LOGI(TAG, "Received PID constants: iteration=%d, converged=%d",
             pid_data->iteration, pid_data->converged);
    
    xSemaphoreGive(g_telem.mutex);
    return ESP_OK;
}

// Get current status
telemetry_status_t telemetry_get_status(void) {
    return g_telem.status;
}

// Get statistics
void telemetry_get_stats(uint32_t *samples_sent, uint32_t *responses_recv, uint32_t *errors) {
    xSemaphoreTake(g_telem.mutex, portMAX_DELAY);
    
    if (samples_sent) *samples_sent = g_telem.samples_sent;
    if (responses_recv) *responses_recv = g_telem.responses_received;
    if (errors) *errors = g_telem.errors;
    
    xSemaphoreGive(g_telem.mutex);
}