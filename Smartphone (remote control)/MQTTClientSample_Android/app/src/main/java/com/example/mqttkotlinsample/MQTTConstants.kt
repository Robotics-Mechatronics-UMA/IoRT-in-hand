package com.example.mqttkotlinsample

const val MQTT_SERVER_URI_KEY   = "MQTT_SERVER_URI"
const val MQTT_CLIENT_ID_KEY    = "MQTT_CLIENT_ID"
const val MQTT_USERNAME_KEY     = "MQTT_USERNAME"
const val MQTT_PWD_KEY          = "MQTT_PWD"

//const val MQTT_SERVER_URI       = "tcp://10.94.92.241:1883" //"tcp://broker.hivemq.com:1883"
const val MQTT_SERVER_URI       = "tcp://192.168.1.100:1883" //"tcp://broker.hivemq.com:1883"
const val MQTT_CLIENT_ID        = ""
const val MQTT_USERNAME         = ""
const val MQTT_PWD              = ""

const val MQTT_TEST_TOPIC       = "UR3_subscriber/jointValues"
const val MQTT_TEST_MSG_deg     = "data:[-125.11, -20.31, 35.91, 233.59, -116.29, 154.29]" // JMove
const val MQTT_TEST_MSG_ef     = "data:[0.400, 0.031, -0.021, 2.13, 1.313, -1.131]"  // Pos+Orient
const val MQTT_TEST_MSG     = "data:[1, 0.0, 0.0, 0.0, 0.0, 0.36]"  // Twist
