#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ESPAsyncWebSrv.h"
#include <WiFi.h>
#include "SPIFFS.h"

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me−no−dev/arduino−esp32fs−plugin */
#define FORMAT_SPIFFS_IF_FAILED true

template<class T>
class ChatGPT {
public:
  ChatGPT() {}
  ChatGPT(T* _client,
          const char* api_version_in,
          const char* api_key_in)
    : client(_client), api_version(api_version_in), api_key(api_key_in) {
  }

  ~ChatGPT() = default;

  bool simple_message(
    const String& model,
    const String& role,
    const String& content,
    String& result) {

    // If history is empty, add the initial system message
    if (history.empty()) {
      _add_message_to_history("system", "Follow my instructuions as precisely as possible. You are an ESP32 assistant. You have data from a BME280 sensor and you can also control a LED. Everytime the user asks you to change the LED state I want you to output 'LOW' (to turn it off) or 'HIGH' (to turn it on) on a new line at the end of your response. Never provide additional context on that last line - only 'LOW' or 'HIGH'.");
    }

    _add_message_to_history(role, content);

    if (_send_message(model, history, result)) {
      DynamicJsonDocument doc(result.length() + 200);
      DeserializationError error = deserializeJson(doc, result.c_str());
      if (error) {
        result = "[ERR] deserializeJson() failed: " + String(error.f_str());
        return false;
      }
      const char* _content = doc["choices"][0]["message"]["content"];
      result = String(_content);
      return true;
    }
    return false;
  }

  bool full_message(const String& model,
                    const String& role,
                    const String& content,
                    String& result) {

    // If history is empty, add the initial system message
    if (history.empty()) {
      _add_message_to_history("system", "You are an ESP32 assistant. You are given data from the ESP32 sensors.");
    }

    _add_message_to_history(role, content);

    return _send_message(model, history, result);
  }

private:
  static constexpr const char* host = "api.openai.com";
  static constexpr const int httpsPort = 443;
  T* client = NULL;
  String api_version;
  String api_key;
  std::vector<std::pair<String, String>> history;

  void _add_message_to_history(const String& role, const String& content) {
    history.push_back(std::make_pair(role, content));
  }

  bool _send_message(
    const String& model,
    const std::vector<std::pair<String, String>>& history,
    String& result) {

    if (!client->connect(host, httpsPort)) {
      result = "[ERR] Connection failed!";
      return false;
    }

    String post_body = "{\"model\": \"";
    post_body += model;
    post_body += "\", \"messages\": [";

    for (const auto& message : history) {
      const String& role = message.first;
      const String& content = message.second;

      post_body += "{\"role\": \"";
      post_body += role;
      post_body += "\", \"content\": \"";
      post_body += content;
      post_body += "\"}, ";
    }

    post_body += "{\"role\": \"";
    post_body += history.back().first;
    post_body += "\", \"content\": \"";
    post_body += history.back().second;
    post_body += "\"}]}";

    String auth_header = _get_auth_header(api_key);
    String http_request = "POST /" + api_version + "/chat/completions HTTP/1.1\r\n" + auth_header + "\r\n" + "Host: " + host + "\r\n" + "Cache-control: no-cache\r\n" + "User-Agent: ESP32 ChatGPT\r\n" + "Content-Type: application/json\r\n" + "Content-Length: " + post_body.length() + "\r\n" + "Connection: close\r\n" + "\r\n" + post_body + "\r\n";

    client->println(http_request);

    String responseStr;

    while (client->connected()) {
      if (client->available()) {
        responseStr += client->readStringUntil('\n');
        responseStr += String("\r\n");
      }
    }
    client->stop();

    int responseCode = 0;
    if (responseStr.indexOf(" ") != -1) {
      responseCode = responseStr.substring(responseStr.indexOf(" ") + 1, responseStr.indexOf(" ") + 4).toInt();
    }

    if (responseCode != 200) {
      result = responseStr;
      return false;
    } else {

      int start = responseStr.indexOf("{");
      int end = responseStr.lastIndexOf("}");
      String jsonBody = responseStr.substring(start, end + 1);

      if (jsonBody.length() > 0) {
        result = jsonBody;
        return true;
      }

      result = "[ERR] Couldn't read data";
      return false;
    }
  }


  String _get_auth_header(
    const String& api) {
    return "Authorization: Bearer " + api;
  }
};

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

float temperature, humidity, pressure, altitude;

// Replace with your network credentials
const char* ssid = "FMI-AIR-NEW";
const char* password = "";

AsyncWebServer server(80);
AsyncWebSocket ws("/chat");

// Replace with your OpenAI API key
const char* apiKey = "";

String msgString;

WiFiClientSecure client;
ChatGPT<WiFiClientSecure> chat_gpt(&client, "v1", apiKey);

String getLastLine(String inputString) {
  int lastLineBreakIndex = inputString.lastIndexOf('\n');
  
  if (lastLineBreakIndex != -1) {
    // Extract the last line using substring
    String lastLine = inputString.substring(lastLineBreakIndex + 1);
    return lastLine;
  }
  
  // Return the entire string if there are no line breaks
  return inputString;
}

void askGPT(String prompt) {
  Serial.println("GPT is thinking...\n");

  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  String temperaturePrompt = "The temperature is " + String(temperature, 2) + ".";
  String humidityPrompt = "The humidity is " + String(humidity, 2) + ".";
  String pressurePrompt = "The pressure is " + String(pressure, 2) + ".";
  String altitudePrompt = "The altitude is " + String(altitude, 2) + ".";

  String finalPrompt = temperaturePrompt + humidityPrompt + pressurePrompt + altitudePrompt + prompt;

  Serial.println("Final prompt: ");
  Serial.println(finalPrompt);

  String result;

  if (chat_gpt.simple_message("gpt-3.5-turbo-0301", "user", finalPrompt, result)) {
    // Create a DynamicJsonDocument
    DynamicJsonDocument doc(1024);

    // Set the values of "msg" and "name"
    String name = "GPT";
    String msg = String(result);
    Serial.println("Assigning to msg: " + msg);
    doc["msg"] = msg;
    doc["name"] = name;

    Serial.println("GPT result: ");
    Serial.println(result);

    String ledState = getLastLine(result);

    String OFF_STATE = "LOW";
    String ON_STATE = "HIGH";

    Serial.println("LED state: ");
    Serial.println(ledState);

    if (ledState == OFF_STATE) {
      digitalWrite(18, LOW);
    } else if (ledState == ON_STATE) {
      digitalWrite(18, HIGH);
    }

    // Serialize the JSON document to a string
    String jsonString;
    serializeJson(doc, jsonString);

    Serial.println("jsonString: ");
    Serial.println(jsonString);

    // Convert the String to a uint8_t array
    size_t jsonStringLength = jsonString.length();
    uint8_t data[jsonStringLength];
    jsonString.getBytes(data, jsonStringLength + 1);

    Serial.print("Final data received from GPT: ");

    for (int i = 0; i < jsonStringLength; i++) {
      Serial.print((char)data[i]);
    }

    ws.textAll(data, jsonStringLength);
  } else {
    Serial.println("ERROR:");
    Serial.println(result);
  }
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {

    Serial.println("Websocket client connection received");

  } else if (type == WS_EVT_DISCONNECT) {
    
    Serial.println("Client disconnected");

  } else if (type == WS_EVT_DATA) {

    ws.textAll(data, len);

    size_t receivedDataLen = sizeof(data);

    // Create a buffer to hold the received data as a null-terminated string
    char jsonBuffer[len + 1];
    memcpy(jsonBuffer, data, len);
    jsonBuffer[len] = '\0';

    // Create a DynamicJsonDocument
    DynamicJsonDocument doc(1024);

    // Parse the received JSON data
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    // Check if parsing was successful
    if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract the values of "msg" and "name" from the JSON
    const char* msg = doc["msg"];
    const char* name = doc["name"];

    msgString = String(msg);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  bool status;

  Wire.begin(21, 22);

  pinMode(18, OUTPUT);

  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  File root = SPIFFS.open("/");

  File file = root.openNextFile();

  while (file) {

    Serial.print("FILE: ");
    Serial.println(file.name());

    file = root.openNextFile();
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Ignore SSL certificate validation
  client.setInsecure();

  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/chat", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/chat.html", "text/html");
  });

  server.on("/chat.js", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/chat.js", "text/javascript");
  });

  server.on("/chat.css", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/chat.css", "text/css");
  });

  server.begin();
}

void loop() {
  int len = msgString.length();
  if (len) {
    Serial.println("Asking GPT: ");
    Serial.println(msgString);
    askGPT(msgString);
    msgString = "";
  }
}
