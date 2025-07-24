
package com.example.mqttkotlinsample
import android.annotation.SuppressLint
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.Gravity
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.*
import androidx.activity.OnBackPressedCallback
import androidx.annotation.RequiresApi
import androidx.fragment.app.Fragment
import androidx.navigation.fragment.findNavController
import org.eclipse.paho.client.mqttv3.*


class ClientFragmentRT : Fragment() {
    private lateinit var mqttClient: MQTTClient




    // val topic_start = "UR3_publisher/jointValues" //"UR3_subscriber/jointValues"
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        activity?.onBackPressedDispatcher?.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                if (mqttClient.isConnected()) {
                    // Disconnect from MQTT Broker
                    mqttClient.disconnect(object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            Log.d(this.javaClass.name, "Disconnected")

                            Toast.makeText(
                                context,
                                "MQTT Disconnection success",
                                Toast.LENGTH_SHORT
                            ).show()

                            // Disconnection success, come back to Connect Fragment

                            findNavController().navigate(com.example.mqttkotlinsample.R.id.action_clientFragmentRT_to_ConnectFragment2)   //////////////////////
                            // Parece que el problema es que no encontraba el contexto: com.example.mqttkotlinsample.

                        }

                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to disconnect")
                        }
                    })
                } else {
                    Log.d(this.javaClass.name, "Impossible to disconnect, no server connected")
                }
            }
        })


    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        return inflater.inflate(
            com.example.mqttkotlinsample.R.layout.fragment_client_rt,
            container,
            false
        )  //com.example.mqttkotlinsample.R.id.action_clientFragmentRT_to_ConnectFragment2
    }

    @SuppressLint("SetTextI18n")
    @RequiresApi(Build.VERSION_CODES.O)
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        // Get arguments passed by ConnectFragment
        val serverURI = arguments?.getString(MQTT_SERVER_URI_KEY)
        val clientId = arguments?.getString(MQTT_CLIENT_ID_KEY)
        val username = arguments?.getString(MQTT_USERNAME_KEY)
        val pwd = arguments?.getString(MQTT_PWD_KEY)

        val baseValues = view?.findViewById(R.id.baseValue) as TextView
        val shoulderValues = view?.findViewById(R.id.shoulderValue) as TextView
        val elbowValues = view?.findViewById(R.id.elbowValue) as TextView
        val wrist1Values = view?.findViewById(R.id.wrist1Value) as TextView
        val wrist2Values = view?.findViewById(R.id.wrist2Value) as TextView
        val wrist3Values = view?.findViewById(R.id.wrist3Value) as TextView

        var previousJ1 = 0
        var previousJ2 = 0
        var previousJ3 = 0
        var previousJ4 = 0
        var previousJ5= 0
        var previousJ6 = 0


        val seekj1 =
            view.findViewById<SeekBar>(com.example.mqttkotlinsample.R.id.seekBarJ1) //com.example.mqttkotlinsample.R.id.seekBarJ1. Antes funcionaba con R.id.seekBarJ1
        seekj1.min = -360 // Deben ser valores reales.
        seekj1.max = 360
        //val topic_j1  = "base"
        val seekj2 = view.findViewById<SeekBar>(com.example.mqttkotlinsample.R.id.seekBarJ2)
        seekj2.min = -360
        seekj2.max = 360
        //val topic_j2 = "shoulder"
        val seekj3 = view.findViewById<SeekBar>(com.example.mqttkotlinsample.R.id.seekBarJ3)
        seekj3.min = -360
        seekj3.max = 360
        //val topic_j3 = "elbow"
        val seekj4 = view.findViewById<SeekBar>(com.example.mqttkotlinsample.R.id.seekBarJ4)
        seekj4.min = -360
        seekj4.max = 360
        //val topic_j4 = "wrist1"
        val seekj5 = view.findViewById<SeekBar>(com.example.mqttkotlinsample.R.id.seekBarJ5)
        seekj5.min = -360
        seekj5.max = 360
        //val topic_j5 = "wrist2"
        val seekj6 = view.findViewById<SeekBar>(com.example.mqttkotlinsample.R.id.seekBarJ6)
        seekj6.min = -360
        seekj6.max = 360
        //val topic_j6 = "wrist3"

        seekj1.progress = 0
        baseValues.setText(seekj1.progress.toString() +" %")
        seekj2.progress = -90
        shoulderValues.setText(seekj2.progress.toString() +" %")
        seekj3.progress = 0
        elbowValues.setText(seekj3.progress.toString() +" %")
        seekj4.progress = -90
        wrist1Values.setText(seekj4.progress.toString() +" %")
        seekj5.progress = 0
        wrist2Values.setText(seekj5.progress.toString() +" %")
        seekj6.progress = 0
        wrist3Values.setText(seekj6.progress.toString() +" %")

        val k = 3.141692 / 180
        val r = 1/k

        // Check if passed arguments are valid
        if (serverURI != null &&
            clientId != null &&
            username != null &&
            pwd != null
        ) {
            // Open MQTT Broker communication
            mqttClient = MQTTClient(context, serverURI, clientId)

            // Connect and login to MQTT Broker
            mqttClient.connect(username,
                pwd,
                object : IMqttActionListener {
                    override fun onSuccess(asyncActionToken: IMqttToken?) {
                        Log.d(this.javaClass.name, "Connection success")

                        // Toast.makeText(context, "MQTT Connection success", Toast.LENGTH_SHORT).show()
                    }

                    override fun onFailure(asyncActionToken: IMqttToken?, exception: Throwable?) {
                        Log.d(this.javaClass.name, "Connection failure: ${exception.toString()}")

                        Toast.makeText(
                            context,
                            "MQTT Connection fails: ${exception.toString()}",
                            Toast.LENGTH_SHORT
                        ).show()

                        // Come back to Connect Fragment
                        //findNavController().navigate(R.id.action_clientFragmentRT_to_ConnectFragment2)
                        findNavController().navigate(com.example.mqttkotlinsample.R.id.action_clientFragmentRT_to_ConnectFragment2)
                    }
                },
                object : MqttCallback {
                    override fun messageArrived(
                        topic: String?,
                        message: MqttMessage?
                    ) {   // This is a client thread
                        val msg = "Receive message: ${message.toString()} from topic: $topic"
                        Log.d(this.javaClass.name, msg)

                        //Toast.makeText(context, msg, Toast.LENGTH_SHORT).show()
                        if (topic == "UR3_publisher/jointValues")   //[0.523615,-1.57085,0,-1.57085,0,0]
                        {
                            val mensaje = message.toString()
                            val x = mensaje.substringBeforeLast("]")
                                .substringAfter("[") //-253.25778803793574, -87.62338492268464, 54.72294059979308, -59.25845034192169, -90.38223550619875, -204.09040152142617
                            val y: List<String> = x.split(",")
                            seekj1.progress = (r * y[0].toFloat()).toInt()
                            seekj2.progress = (r * y[1].toFloat()).toInt()
                            seekj3.progress = (r * y[2].toFloat()).toInt()
                            seekj4.progress = (r * y[3].toFloat()).toInt()
                            seekj5.progress = (r * y[4].toFloat()).toInt()
                            seekj6.progress = (r * y[5].toFloat()).toInt()



                        } else
                            Log.d(this.javaClass.name, "Nothing ")
                        if (topic == "UR3_publisher/ActualjointValues")   //        //y[-253.25778803793574, -87.62201888456644, 54.72015388203195, -59.25708430380349, -90.38155248713966, -204.09176755954437]
                        {
                            val mensaje = message.toString()
                            val x = mensaje.substringBeforeLast("]")
                                .substringAfter("[") //-253.25778803793574, -87.62338492268464, 54.72294059979308, -59.25845034192169, -90.38223550619875, -204.09040152142617
                            val y: List<String> = x.split(",")
                            seekj1.progress = (y[0].toFloat()).toInt()
                            seekj2.progress = (y[1].toFloat()).toInt()
                            seekj3.progress = (y[2].toFloat()).toInt()
                            seekj4.progress = (y[3].toFloat()).toInt()
                            seekj5.progress = (y[4].toFloat()).toInt()
                            seekj6.progress = (y[5].toFloat()).toInt()


                        } else
                            Log.d(this.javaClass.name, "Nothing ")

                } //messageArrived

                    override fun connectionLost(cause: Throwable?) {
                        Log.d(this.javaClass.name, "Connection lost ${cause.toString()}")
                    }

                    override fun deliveryComplete(token: IMqttDeliveryToken?) {
                        Log.d(this.javaClass.name, "Delivery complete")
                    }
                })
        } else {
            // Arguments are not valid, come back to Connect Fragment
            findNavController().navigate(com.example.mqttkotlinsample.R.id.action_clientFragmentRT_to_ConnectFragment2)
        }
        var change = 0 // To decide what strings should be transmitted (via MQTT or ROS).
        var movement =
            0 // To decide what movement shoudl be executed (point or joinValues of the UR3).
        var dt = 0
        var pr = 0



        /*
        edit1=(EditText)findViewById(R.id.edit1);
        float value= (float) (progress / 1000.0);
        edit1.setText((value)+"\n");
        */




        val topic_ptp1 = "pose_ptp1/jointValues" // mover punto
        val topic_lin1 = "pose_lin1/coordinates"
        val topic_robot = "UR3_subscriber/jointValues" // mover articulaciones del robot (una a una)
        var topic = topic_robot

        //SEEKBARJ1
        /*
        The SeekBar class is a subclass of ProgressBar, which contains a getProgress() method.
        Calling PRICEbar.getProgress() will return the value you are looking for.
        */



        seekj1?.setOnSeekBarChangeListener(object :
            SeekBar.OnSeekBarChangeListener {

            override fun onProgressChanged(
                seekj1: SeekBar,
                progress: Int, fromUser: Boolean
            ) {
                // write custom code for progress is change
                var message = ""
                if (change == 1) {
                    message =
                        "data:[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                } else {
                    message =
                        "[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"

                    baseValues.setText(seekj1.progress.toString() + " %")
                    shoulderValues.setText(seekj2.progress.toString() + " %")
                    elbowValues.setText(seekj3.progress.toString() + " %")
                    wrist1Values.setText(seekj4.progress.toString() + " %")
                    wrist2Values.setText(seekj5.progress.toString() + " %")
                    wrist3Values.setText(seekj6.progress.toString() + " %")

                }

                //textView.setY(100); just added a value set this properly using screen with height aspect ratio , if you do not set it by default it will be there below seek bar
                if (mqttClient.isConnected()) {

                    // Si el checkbox está seleccionado, entonces se envía por este topic (mover robot), si no por el otro (mover punto)
                    mqttClient.publish(topic, message, 1, false, object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            val msg = "Publish message: $message to topic: $topic"
                            Log.d(this.javaClass.name, msg)
                        }


                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to publish message to topic")
                        }
                    })


                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }

            } // onProgressChanged

            override fun onStartTrackingTouch(seekj1: SeekBar) {
                // write custom code for progress is started

            }

            override fun onStopTrackingTouch(seekj1: SeekBar) {
                // write custom code for progress is stopped

                val mytoast = Toast.makeText(
                    context,
                    "Value: " + seekj1.progress + "º",
                    Toast.LENGTH_SHORT
                )
/*
                if (mqttClient.isConnected()) {
                    mqttClient.subscribe(topic,
                        1,
                        object : IMqttActionListener {
                            override fun onSuccess(asyncActionToken: IMqttToken?) {

                                val msg = topic
                                val separated = msg.split(",").toTypedArray()
                                val separated2 = separated[0].split("[").toTypedArray()
                                seekj1.progress =
                                    (separated2[1].toString().toInt()*360/3.141).toInt()  // Hay que ver la forma de obtener cada uno de los valores enteros
                                //val msg = "Subscribed to: $topic_start"
                                //Log.d(this.javaClass.name, msg)

                                //Toast.makeText(context, separated2.toString(), Toast.LENGTH_SHORT).show()
                            }

                            override fun onFailure(asyncActionToken: IMqttToken?, exception: Throwable?) {
                                Log.d(this.javaClass.name, "Failed to publish message to topic")
                            }


                        })
                }

 */

                //mytoast.setGravity(Gravity.TOP or Gravity.LEFT, 0, 0)
                mytoast.setGravity(Gravity.TOP, 55, 18)
                mytoast.show()


            }
        })

        /*
         *   Toast toast = new Toast(getContext());
             View toast_view = LayoutInflater.from(getContext()).inflate(R.layout.toast_text, null);
             toast.setView(toast_view);
             toast.setDuration(Toast.LENGTH_SHORT);
             toast.setGravity(Gravity.CENTER, 0, 0);
             toast.show();
         * */

        //SEEKBARJ2


        seekj2?.setOnSeekBarChangeListener(object :
            SeekBar.OnSeekBarChangeListener {

            override fun onProgressChanged(
                seekj2: SeekBar,
                progress: Int, fromUser: Boolean
            ) {
                // write custom code for progress is changed
                var message = ""
                if (change == 1) { //ROS
                    message =
                        "data:[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                } else {
                    message =
                        "[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                    baseValues.setText(seekj1.progress.toString() + " %")
                    shoulderValues.setText(seekj2.progress.toString() + " %")
                    elbowValues.setText(seekj3.progress.toString() + " %")
                    wrist1Values.setText(seekj4.progress.toString() + " %")
                    wrist2Values.setText(seekj5.progress.toString() + " %")
                    wrist3Values.setText(seekj6.progress.toString() + " %")
                }



                if (mqttClient.isConnected()) {
                    mqttClient.publish(topic, message, 1, false, object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            val msg = "Publish message: $message to topic: $topic"
                            Log.d(this.javaClass.name, msg)
                        }

                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to publish message to topic")
                        }
                    })
                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }
            }

            override fun onStartTrackingTouch(seekj2: SeekBar) {
                // write custom code for progress is started
            }

            override fun onStopTrackingTouch(seekj2: SeekBar) {
                // write custom code for progress is stopped
                //makeText takes 3 parameters:
                // the application context,
                // the text that should appear on the screen,
                // and the duration that the toast should remain on the screen.
                val mytoast = Toast.makeText(
                    context,
                    "Progress is: " + seekj2.progress + "%",
                    Toast.LENGTH_SHORT
                )
                mytoast.setGravity(Gravity.TOP or Gravity.LEFT, 0, 0)
                mytoast.show()
            }
        })


        //SEEKBARJ3


        seekj3?.setOnSeekBarChangeListener(object :
            SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekj3: SeekBar,
                progress: Int, fromUser: Boolean
            ) {
                // write custom code for progress is changed

                var message = ""
                if (change == 1) { //ROS
                    message =
                        "data:[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                } else {
                    message =
                        "[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                    baseValues.setText(seekj1.progress.toString() + " %")
                    shoulderValues.setText(seekj2.progress.toString() + " %")
                    elbowValues.setText(seekj3.progress.toString() + " %")
                    wrist1Values.setText(seekj4.progress.toString() + " %")
                    wrist2Values.setText(seekj5.progress.toString() + " %")
                    wrist3Values.setText(seekj6.progress.toString() + " %")
                }


                if (mqttClient.isConnected()) {
                    mqttClient.publish(topic, message, 1, false, object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            val msg = "Publish message: $message to topic: $topic"
                            Log.d(this.javaClass.name, msg)
                        }

                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to publish message to topic")
                        }
                    })
                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }
            }

            override fun onStartTrackingTouch(seekj3: SeekBar) {
                // write custom code for progress is started
            }

            override fun onStopTrackingTouch(seekj3: SeekBar) {
                // write custom code for progress is stopped
                val mytoast = Toast.makeText(
                    context,
                    "Progress is: " + seekj3.progress + "%",
                    Toast.LENGTH_SHORT
                )
                mytoast.setGravity(Gravity.TOP or Gravity.LEFT, 0, 0)
                mytoast.show()
            }
        })

        //SEEKBARJ4


        seekj4?.setOnSeekBarChangeListener(object :
            SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekj4: SeekBar,
                progress: Int, fromUser: Boolean
            ) {
                // write custom code for progress is changed
                var message = ""
                if (change == 1) { //ROS
                    message =
                        "data:[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                } else {
                    message =
                        "[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                    baseValues.setText(seekj1.progress.toString() + " %")
                    shoulderValues.setText(seekj2.progress.toString() + " %")
                    elbowValues.setText(seekj3.progress.toString() + " %")
                    wrist1Values.setText(seekj4.progress.toString() + " %")
                    wrist2Values.setText(seekj5.progress.toString() + " %")
                    wrist3Values.setText(seekj6.progress.toString() + " %")
                }


                if (mqttClient.isConnected()) {
                    mqttClient.publish(topic, message, 1, false, object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            val msg = "Publish message: $message to topic: $topic"
                            Log.d(this.javaClass.name, msg)
                        }

                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to publish message to topic")
                        }
                    })
                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }
            }

            override fun onStartTrackingTouch(seekj4: SeekBar) {
                // write custom code for progress is started
            }

            override fun onStopTrackingTouch(seekj4: SeekBar) {
                // write custom code for progress is stopped
                val mytoast = Toast.makeText(
                    context,
                    "Progress is: " + seekj4.progress + "%",
                    Toast.LENGTH_SHORT
                )
                mytoast.setGravity(Gravity.TOP or Gravity.LEFT, 0, 0)
                mytoast.show()
            }
        })

        //SEEKBARJ5


        seekj5?.setOnSeekBarChangeListener(object :
            SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekj5: SeekBar,
                progress: Int, fromUser: Boolean
            ) {
                // write custom code for progress is changed
                var message = ""
                if (change == 1) { //ROS
                    message =
                        "data:[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                } else {
                    message =
                        "[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                    baseValues.setText(seekj1.progress.toString() + " %")
                    shoulderValues.setText(seekj2.progress.toString() + " %")
                    elbowValues.setText(seekj3.progress.toString() + " %")
                    wrist1Values.setText(seekj4.progress.toString() + " %")
                    wrist2Values.setText(seekj5.progress.toString() + " %")
                    wrist3Values.setText(seekj6.progress.toString() + " %")
                }

                if (mqttClient.isConnected()) {
                    mqttClient.publish(topic, message, 1, false, object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            val msg = "Publish message: $message to topic: $topic"
                            Log.d(this.javaClass.name, msg)
                        }

                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to publish message to topic")
                        }
                    })
                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }
            }

            override fun onStartTrackingTouch(seekj5: SeekBar) {
                // write custom code for progress is started
            }

            override fun onStopTrackingTouch(seekj5: SeekBar) {
                // write custom code for progress is stopped
                val mytoast = Toast.makeText(
                    context,
                    "Progress is: " + seekj5.progress + "%",
                    Toast.LENGTH_SHORT
                )
                mytoast.setGravity(Gravity.TOP or Gravity.LEFT, 0, 0)
                mytoast.show()
            }
        })

        //SEEKBARJ6

        seekj6?.setOnSeekBarChangeListener(object :
            SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekj6: SeekBar,
                progress: Int, fromUser: Boolean
            ) {
                // write custom code for progress is changed
                //val message = "data:["+(k*seekj1.progress).toString() + ", " + (k*seekj2.progress).toString() + ", " + (k*seekj3.progress).toString() + ", " + (k*seekj4.progress).toString() + ", " + (k*seekj5.progress).toString() + ", " + (k*seekj6.progress).toString() + "]"

                var message = ""
                if (change == 1) { //ROS
                    message =
                        "data:[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                } else {
                    message =
                        "[" + (k * seekj1.progress).toString() + ", " + (k * seekj2.progress).toString() + ", " + (k * seekj3.progress).toString() + ", " + (k * seekj4.progress).toString() + ", " + (k * seekj5.progress).toString() + ", " + (k * seekj6.progress).toString() + "]"
                    baseValues.setText(seekj1.progress.toString() + " %")
                    shoulderValues.setText(seekj2.progress.toString() + " %")
                    elbowValues.setText(seekj3.progress.toString() + " %")
                    wrist1Values.setText(seekj4.progress.toString() + " %")
                    wrist2Values.setText(seekj5.progress.toString() + " %")
                    wrist3Values.setText(seekj6.progress.toString() + " %")
                }

                if (mqttClient.isConnected()) {
                    mqttClient.publish(topic, message, 1, false, object : IMqttActionListener {
                        override fun onSuccess(asyncActionToken: IMqttToken?) {
                            val msg = "Publish message: $message to topic: $topic"
                            Log.d(this.javaClass.name, msg)
                        }

                        override fun onFailure(
                            asyncActionToken: IMqttToken?,
                            exception: Throwable?
                        ) {
                            Log.d(this.javaClass.name, "Failed to publish message to topic")
                        }
                    })
                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }
            }

            override fun onStartTrackingTouch(seekj6: SeekBar) {
                // write custom code for progress is started
            }

            override fun onStopTrackingTouch(seekj6: SeekBar) {
                // write custom code for progress is stopped
                val mytoast = Toast.makeText(
                    context,
                    "Progress is: " + seekj6.progress + "%",
                    Toast.LENGTH_SHORT
                )
                mytoast.setGravity(Gravity.TOP or Gravity.LEFT, 0, 0)
                mytoast.show()
            }

        })


        /*CHECKBOXES ASOCIADOS A IMAGENES*/

        val im_ros = view.findViewById<ImageView>(com.example.mqttkotlinsample.R.id.rosimg)
        val im_mqtt = view.findViewById<ImageView>(com.example.mqttkotlinsample.R.id.mqttimg)
        val im_point = view.findViewById<ImageView>(com.example.mqttkotlinsample.R.id.point)
        val im_cobot = view.findViewById<ImageView>(com.example.mqttkotlinsample.R.id.cobot)

        val simpleCheckBox1: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.ROScheck) as CheckBox
        val simpleCheckBox2: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.MQTTcheck) as CheckBox
        val simpleCheckBox3: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.POINTcheck) as CheckBox
        val simpleCheckBox4: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.COBOTcheck) as CheckBox

        simpleCheckBox1.setEnabled(false);
        simpleCheckBox2.setEnabled(true);
        simpleCheckBox3.setEnabled(false);
        simpleCheckBox4.setEnabled(true);


// set the current state of a check box

        im_ros.setOnClickListener(
            View.OnClickListener
            //@Override
            {

                // val mytoast = Toast.makeText(context, "Message format for ROS", Toast.LENGTH_SHORT)
                // mytoast.show()
                // Change strings for the data to transmit
                if (change != 1) {
                    change = 1
                    simpleCheckBox1.setChecked(true)
                    simpleCheckBox2.setChecked(false)
                }
            })

        im_mqtt.setOnClickListener(
            View.OnClickListener
            //@Override
            {

                // val mytoast = Toast.makeText(context, "message format for MQTT", Toast.LENGTH_SHORT)
                // mytoast.show()
                // Change strings for the data to transmit
                if (change != 0) {
                    change = 0
                    simpleCheckBox2.setChecked(true)
                    simpleCheckBox1.setChecked(false)
                }
            })

        im_point.setOnClickListener(
            View.OnClickListener
            //@Override
            {

                // val mytoast = Toast.makeText(context, "message format for MQTT", Toast.LENGTH_SHORT)
                // mytoast.show()
                // Change strings for the data to transmit
                if (movement != 1) {
                    movement = 1
                    simpleCheckBox3.setChecked(true)
                    simpleCheckBox4.setChecked(false)
                    topic = topic_ptp1
                    /*

                    val topic_ptp1 = "pose_ptp1/jointValues" // mover punto
                    val topic_lin1 = "pose_lin1/coordinates"
                    val topic_robot = "UR3_subscriber/jointValues" // mover articulaciones del robot (una a una)

                     */
                }
            })

        im_cobot.setOnClickListener(
            View.OnClickListener
            //@Override
            {

                // val mytoast = Toast.makeText(context, "message format for MQTT", Toast.LENGTH_SHORT)
                // mytoast.show()
                // Change strings for the data to transmit
                if (movement != 0) {
                    movement = 0
                    simpleCheckBox3.setChecked(false)
                    simpleCheckBox4.setChecked(true)
                    topic = topic_robot
                }
            })

        view?.findViewById<Button>(com.example.mqttkotlinsample.R.id.auto_move)
            ?.setOnClickListener {

                val topic_auto = "ptp/enabled"
                val message_auto = "1";
                if (mqttClient.isConnected()) {
                    mqttClient.publish(
                        topic_auto,
                        message_auto,
                        1,
                        false,
                        object : IMqttActionListener {
                            override fun onSuccess(asyncActionToken: IMqttToken?) {
                                val msg = "Publish message: $message_auto to topic: $topic_auto"
                                Log.d(this.javaClass.name, msg)
                            }

                            override fun onFailure(
                                asyncActionToken: IMqttToken?,
                                exception: Throwable?
                            ) {
                                Log.d(this.javaClass.name, "Failed to publish message to topic")
                            }
                        })


                } else {
                    Log.d(this.javaClass.name, "Impossible to publish, no server connected")
                }

                //Toast.makeText(context, "BOTON PULSADO", Toast.LENGTH_SHORT).show()


            } //boton pulsado

        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////
        /* OBTENER VALORES DEL GEMELO DIGITAL*/
        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////

        val simpleCheckBox5: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.getDTpose) as CheckBox

        val simpleCheckBox6: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.getPRpose) as CheckBox

        val simpleCheckBox7: CheckBox =
            view.findViewById(com.example.mqttkotlinsample.R.id.getPreviousDTpose) as CheckBox


        simpleCheckBox5.setOnClickListener {
            val topic2 = "UR3_publisher/jointValues"
            simpleCheckBox5.setEnabled(true);
            if(simpleCheckBox5.isChecked()) {
                // Suscribir al topic MQTT: UR3_publisher/jointValues
                simpleCheckBox5.setEnabled(true);
                if (mqttClient.isConnected()) {
                    mqttClient.subscribe(topic2,
                        1,
                        object : IMqttActionListener {
                            override fun onSuccess(asyncActionToken: IMqttToken?) {

                            }

                            override fun onFailure(
                                asyncActionToken: IMqttToken?,
                                exception: Throwable?
                            ) {
                                Log.d(this.javaClass.name, "Failed to subscribe: $topic2")
                            }
                        },
                        object : MqttCallback {
                            override fun messageArrived(
                                topic: kotlin.String,
                                message: org.eclipse.paho.client.mqttv3.MqttMessage
                            ) {
                                val msg =
                                    "Receive message: ${message.toString()} from topic: $topic"

                            }

                            override fun connectionLost(cause: kotlin.Throwable) {
                                Log.d(javaClass.name, "Connection lost ${cause.toString()}")
                            }

                            override fun deliveryComplete(token: org.eclipse.paho.client.mqttv3.IMqttDeliveryToken) {
                                Log.d(javaClass.name, "Delivery complete")
                            }
                        })
                } else {
                    Log.d(this.javaClass.name, "Impossible to subscribe, no server connected")
                }
            }
            else {
                //simpleCheckBox5.setChecked(false)
                previousJ1 = seekj1.progress  // Previos a la teleoperacion
                previousJ2 = seekj2.progress
                previousJ3 = seekj3.progress
                previousJ4 = seekj4.progress
                previousJ5 = seekj5.progress
                previousJ6 = seekj6.progress

                if (mqttClient.isConnected()) {
                    mqttClient.unsubscribe( topic2,
                        object : IMqttActionListener {
                            override fun onSuccess(asyncActionToken: IMqttToken?) {
                                val msg = "Unsubscribed to: $topic2"
                                Log.d(this.javaClass.name, msg)

                                Toast.makeText(context, msg, Toast.LENGTH_SHORT).show()
                            }

                            override fun onFailure(asyncActionToken: IMqttToken?, exception: Throwable?) {
                                Log.d(this.javaClass.name, "Failed to unsubscribe: $topic2")
                            }
                        })
                } else {
                    Log.d(this.javaClass.name, "Impossible to unsubscribe, no server connected")
                }
            }
        }


        simpleCheckBox6.setOnClickListener {
            val topic2 = "UR3_publisher/ActualjointValues"
            simpleCheckBox6.setEnabled(true);
            if(simpleCheckBox6.isChecked()) {
                // Suscribir al topic MQTT: UR3_publisher/jointValues
                simpleCheckBox6.setEnabled(true);
                if (mqttClient.isConnected()) {
                    mqttClient.subscribe(topic2,
                        1,
                        object : IMqttActionListener {
                            override fun onSuccess(asyncActionToken: IMqttToken?) {

                            }

                            override fun onFailure(
                                asyncActionToken: IMqttToken?,
                                exception: Throwable?
                            ) {
                                Log.d(this.javaClass.name, "Failed to subscribe: $topic2")
                            }
                        },
                        object : MqttCallback {
                            override fun messageArrived(
                                topic: kotlin.String,
                                message: org.eclipse.paho.client.mqttv3.MqttMessage
                            ) {
                                val msg =
                                    "Receive message: ${message.toString()} from topic: $topic"

                            }

                            override fun connectionLost(cause: kotlin.Throwable) {
                                Log.d(javaClass.name, "Connection lost ${cause.toString()}")
                            }

                            override fun deliveryComplete(token: org.eclipse.paho.client.mqttv3.IMqttDeliveryToken) {
                                Log.d(javaClass.name, "Delivery complete")
                            }
                        })
                } else {
                    Log.d(this.javaClass.name, "Impossible to subscribe, no server connected")
                }
            }
            else {
                //simpleCheckBox5.setChecked(false)
                if (mqttClient.isConnected()) {
                    mqttClient.unsubscribe( topic2,
                        object : IMqttActionListener {
                            override fun onSuccess(asyncActionToken: IMqttToken?) {
                                val msg = "Unsubscribed to: $topic2"
                                Log.d(this.javaClass.name, msg)

                                Toast.makeText(context, msg, Toast.LENGTH_SHORT).show()
                            }

                            override fun onFailure(asyncActionToken: IMqttToken?, exception: Throwable?) {
                                Log.d(this.javaClass.name, "Failed to unsubscribe: $topic2")
                            }
                        })
                } else {
                    Log.d(this.javaClass.name, "Impossible to unsubscribe, no server connected")
                }
            }


        } //simplecheckbox6


        simpleCheckBox7.setOnClickListener {
            seekj1.progress  = previousJ1
            seekj2.progress =  previousJ2
            seekj3.progress = previousJ3
            seekj4.progress = previousJ4
            seekj5.progress = previousJ5
            seekj6.progress = previousJ6
        }


    } //onViewCreated

        //topic from ROS (actual joints values):
        //y[-253.25778803793574, -87.62201888456644, 54.72015388203195, -59.25708430380349, -90.38155248713966, -204.09176755954437]
}








