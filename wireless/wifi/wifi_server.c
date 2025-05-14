/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include <esp_http_server.h> 
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_check.h"
#include <time.h>
#include <sys/time.h>

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 */

#include "ascent_r2_hardware_definition.h"  // Hardware definitions
#include "driver_pyro.h"


static const char *TAG = "example";

static bool deploy_drogues()
{   
    printf("\n\n DROGUES!!!! \n\n");
    bool cont;

    pyro_activate(PYRO_CHANNEL_1, 250, 1);

#ifdef LED_PYRO
    return true;
#else
    return false;
#endif
}

static bool deploy_mains()
{
    printf("\n\n MAINS!!!! \n\n");
    
    pyro_activate(PYRO_CHANNEL_2, 250, 1);

#ifdef LED_PYRO
    return true;
#else
    return false;
#endif
}

/* An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    // char*  buf;
    // size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    // buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    
    const char* resp_str =
"<html>"
"<head>"
"    <style>"
"        body {"
"            display: flex;"
"            justify-content: center;"
"            align-items: center;"
"            height: 100vh;"
"            margin: 0;"
"            font-family: Arial, sans-serif;"
"            background-color: #f8f9fa;"
"        }"
"        .container {"
"            text-align: center;"
"        }"
"        .display-4 {"
"            font-size: 3rem;"
"            color: #343a40;"
"        }"
"        .lead {"
"            font-size: 1.25rem;"
"            color: #6c757d;"
"        }"
"        .btn {"
"            padding: 10px 20px;"
"            font-size: 1.25rem;"
"            color: white;"
"            background-color: #007bff;"
"            border: none;"
"            border-radius: 5px;"
"            text-decoration: none;"
"        }"
"        .btn:hover {"
"            background-color: #0056b3;"
"        }"
"        .btn2 {"
"            padding: 10px 20px;"
"            font-size: 1.25rem;"
"            color: white;"
"            background-color: #ff00f7;"
"            border: none;"
"            border-radius: 5px;"
"            text-decoration: none;"
"        }"
"        .btn2:hover {"
"            background-color: #b30098;"
"        }"
"    </style>"
"    <script>"
"        function sendRequest(endpoint) {"
"            if (confirm('fire a pyro?')) {"
"                fetch(endpoint)"
"                    .then(response => {"
"                        if (response.ok) {"
"                        } else {"
"                            alert('connection error');"
"                        }"
"                    })"
"            }"
"        }"
"    </script>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1 class='display-4'>ASCENT Pop Test</h1>"
"        <a class='btn' href='#' role='button' onclick=\"sendRequest('/apogee')\">Apogee</a>"
"        <a class='btn2' href='#' role='button' onclick=\"sendRequest('/mains')\">Mains</a>"
"    </div>"
"</body>"
"</html>";

    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static esp_err_t apogee_get_handler(httpd_req_t *req)
{   
    const char* resp_str = "Apogee Fired!";
    deploy_drogues();
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static esp_err_t mains_get_handler(httpd_req_t *req)
{   
    const char* resp_str = "Mains Fired!";
    deploy_mains();
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
};

static const httpd_uri_t apogee = {
    .uri       = "/apogee",
    .method    = HTTP_GET,
    .handler   = apogee_get_handler,
};

static const httpd_uri_t mains = {
    .uri       = "/mains",
    .method    = HTTP_GET,
    .handler   = mains_get_handler,
};


static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &apogee);
        httpd_register_uri_handler(server, &mains);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

#if !CONFIG_IDF_TARGET_LINUX
static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
#endif // !CONFIG_IDF_TARGET_LINUX

void setup_webserver(void)
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));

    /* Start the server for the first time */
    server = start_webserver();
}