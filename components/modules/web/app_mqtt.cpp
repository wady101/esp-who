// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "app_httpd.hpp"

#include <list>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "app_mdns.h"
#include "sdkconfig.h"

#include "who_camera.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "camera_httpd";
#endif

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static bool gReturnFB = true;

static int8_t detection_enabled = 0;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}

/* MQTT init code starts here */
// Access IoTHub via MQTTT Protocol with mosquitto library (no IoT SDK used)
//

#include <cstdio>
#include <fstream>
#include "mosquitto.h"

// CONNECTION information to complete
#define IOTHUBNAME "<your IoT Hub name>"
#define DEVICEID "<your device Id>"
#define PWD "SharedAccessSignature sr=[yourIoTHub].azure-devices.net%2Fdevices%2F[nameofyourdevice]&sig=[tokengeneratedforyourdevice]"


#define CERTIFICATEFILE "IoTHubRootCA_Baltimore.pem"

// computed Host Username and Topic
#define USERNAME IOTHUBNAME ".azure-devices.net/" DEVICEID "/?api-version=2018-06-30"
#define PORT 8883
#define HOST IOTHUBNAME ".azure-devices.net"
#define TOPIC "devices/" DEVICEID "/messages/events/"

// Note
// Certificate
//  Server certs available here for download: https://raw.githubusercontent.com/Azure/azure-iot-sdk-c/master/certs/certs.c
// 
// PWD
//  Generated via Azure CLI, Device explorer or VSCode Azure IoT extension (Generate SAS Token for device)
//  az iot hub generate-sas-token -d EM_MXC3166 -n EricmittHub
// 
// Username
//  Username format for MQTT connection to Hub: $hub_hostname/$device_id/?api-version=2018-06-30"

// Callback functions
void connect_callback(struct mosquitto* mosq, void* obj, int result)
{
	printf("Connect callback returned result: %s\r\n", mosquitto_strerror(result));

	if (result == MOSQ_ERR_CONN_REFUSED)
		printf("Connection refused. Please check DeviceId, IoTHub name or if your SAS Token has expired.\r\n");
}

void publish_callback(struct mosquitto* mosq, void* userdata, int mid)
{
	printf("Publish OK. Now disconnecting the client.\r\n");
	mosquitto_disconnect(mosq);
}

int mosquitto_error(int rc, const char* message = NULL)
{
	printf("Error: %s\r\n", mosquitto_strerror(rc));

	if (message != NULL)
	{
		printf("%s\r\n", message);
	}

	mosquitto_lib_cleanup();
	return rc;
}

/* MQTT init code ends here */
static esp_err_t capture_handler(void)
{
    camera_fb_t *frame = NULL;
    esp_err_t res = ESP_OK;

    if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
    {

        char ts[32];
        snprintf(ts, 32, "%ld.%06ld", frame->timestamp.tv_sec, frame->timestamp.tv_usec);

        /////// MQTT code from here 
        int rc;
      	printf("Using MQTT to send message to %s.\r\n", HOST);
      
      	// init the mosquitto lib
      	mosquitto_lib_init();
      
      	// create the mosquito object
      	struct mosquitto* mosq = mosquitto_new(DEVICEID, false, NULL);
      
      	// add callback functions
      	mosquitto_connect_callback_set(mosq, connect_callback);
      	mosquitto_publish_callback_set(mosq, publish_callback);
      
      	// set mosquitto username, password and options
      	mosquitto_username_pw_set(mosq, USERNAME, PWD);
      
      	// specify the certificate to use
      	std::ifstream infile(CERTIFICATEFILE);
      	bool certExists = infile.good();
      	infile.close();
      	if (!certExists)
      	{
      		printf("Warning: Could not find file '%s'! The mosquitto loop may fail.\r\n", CERTIFICATEFILE);
      	}
      
      	printf("Using certificate: %s\r\n", CERTIFICATEFILE);
      	mosquitto_tls_set(mosq, CERTIFICATEFILE, NULL, NULL, NULL, NULL);
      
      	// specify the mqtt version to use
      	int* option = new int(MQTT_PROTOCOL_V311);
      	rc = mosquitto_opts_set(mosq, MOSQ_OPT_PROTOCOL_VERSION, option);
      	if (rc != MOSQ_ERR_SUCCESS)
      	{
      		return mosquitto_error(rc, "Error: opts_set protocol version");
      	}
      	else
      	{
      		printf("Setting up options OK\r\n");
      	}
      
      	// connect
      	printf("Connecting...\r\n");
      	rc = mosquitto_connect(mosq, HOST, PORT, 10);
      	if (rc != MOSQ_ERR_SUCCESS)
      	{
      		return mosquitto_error(rc);
      	}
      
      	printf("Connect returned OK\r\n");
      
      	int msgId = 42;
      
      	/////// MQTT code ends here 

        // size_t fb_len = 0;
        if (frame->format == PIXFORMAT_JPEG)
        {
            // once connected, we can publish (send) a Telemetry message
      	    printf("Publishing....\r\n");
      	    rc = mosquitto_publish(mosq, &msgId, TOPIC, frame->len,(const char *)frame->buf , 1, true);
      	    if (rc != MOSQ_ERR_SUCCESS)
      	    {
      	    	return mosquitto_error(rc);
      	    }
      
      	    printf("Publish returned OK\r\n");
      
      	    // according to the mosquitto doc, a call to loop is needed when dealing with network operation
      	    // see https://github.com/eclipse/mosquitto/blob/master/lib/mosquitto.h
      	    printf("Entering Mosquitto Loop...\r\n");
      	    mosquitto_loop_forever(mosq, -1, 1);
      
      	    mosquitto_lib_cleanup();
        }
        else
        {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(frame, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
            // fb_len = jchunk.len;
        }

        if (xQueueFrameO)
        {
            xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
        }
        else if (gReturnFB)
        {
            esp_camera_fb_return(frame);
        }
        else
        {
            free(frame);
        }
    }
    else
    {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }

    return res;
}


void register_mqttd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = return_fb;

    auto res = capture_handler();

    
}
