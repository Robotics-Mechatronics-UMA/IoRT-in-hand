package com.example.mqttkotlinsample

import android.content.Intent
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText
import androidx.core.os.bundleOf
import androidx.fragment.app.Fragment
import androidx.navigation.fragment.findNavController

class ConnectFragment : Fragment() {

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_connect, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)


        view.findViewById<Button>(R.id.ROSM).setOnClickListener {
            val launchIntent: Intent? =
                context?.packageManager?.getLaunchIntentForPackage("com.schneewittchen.rosandroid") // To launch another app
            startActivity(launchIntent)
        }

        view.findViewById<Button>(R.id.UMA).setOnClickListener {
            val launchIntent: Intent? =
                context?.packageManager?.getLaunchIntentForPackage("com.example.umaSSRR") // To launch another app
            startActivity(launchIntent)
        }

        view.findViewById<Button>(R.id.button_clean).setOnClickListener {
            // Clean values in edit texts
            view.findViewById<EditText>(R.id.edittext_server_uri).setText("")
            view.findViewById<EditText>(R.id.edittext_client_id).setText("")
            view.findViewById<EditText>(R.id.edittext_username).setText("")
            view.findViewById<EditText>(R.id.edittext_password).setText("")
            view.findViewById<EditText>(R.id.robot_access).setText("")
        }


        view.findViewById<Button>(R.id.button_prefill).setOnClickListener {
            // Set default values in edit texts
            view.findViewById<EditText>(R.id.edittext_server_uri).setText(MQTT_SERVER_URI)
            view.findViewById<EditText>(R.id.edittext_client_id).setText(MQTT_CLIENT_ID)
            view.findViewById<EditText>(R.id.edittext_username).setText(MQTT_USERNAME)
            view.findViewById<EditText>(R.id.edittext_password).setText(MQTT_PWD)
        }

        view.findViewById<Button>(R.id.cam).setOnClickListener {
            val serverURIFromEditText =
                view.findViewById<EditText>(R.id.edittext_server_uri).text.toString()
            val clientIDFromEditText =
                view.findViewById<EditText>(R.id.edittext_client_id).text.toString()
            val usernameFromEditText =
                view.findViewById<EditText>(R.id.edittext_username).text.toString()
            val pwdFromEditText =
                view.findViewById<EditText>(R.id.edittext_password).text.toString()

            val mqttCredentialsBundle = bundleOf(
                MQTT_SERVER_URI_KEY to serverURIFromEditText,
                MQTT_CLIENT_ID_KEY to clientIDFromEditText,
                MQTT_USERNAME_KEY to usernameFromEditText,
                MQTT_PWD_KEY to pwdFromEditText
            )

            findNavController().navigate(
                R.id.action_ConnectFragment_to_clientFragmentDcam,
                mqttCredentialsBundle
            )
        }

        view.findViewById<Button>(R.id.button_connect).setOnClickListener {
            val serverURIFromEditText =
                view.findViewById<EditText>(R.id.edittext_server_uri).text.toString()
            val clientIDFromEditText =
                view.findViewById<EditText>(R.id.edittext_client_id).text.toString()
            val usernameFromEditText =
                view.findViewById<EditText>(R.id.edittext_username).text.toString()
            val pwdFromEditText =
                view.findViewById<EditText>(R.id.edittext_password).text.toString()

            val mqttCredentialsBundle = bundleOf(
                MQTT_SERVER_URI_KEY to serverURIFromEditText,
                MQTT_CLIENT_ID_KEY to clientIDFromEditText,
                MQTT_USERNAME_KEY to usernameFromEditText,
                MQTT_PWD_KEY to pwdFromEditText
            )

            if (view.findViewById<EditText>(R.id.robot_access).text.toString() == "Cobot") {
                findNavController().navigate(
                    R.id.action_ConnectFragment_to_ClientFragment,
                    mqttCredentialsBundle
                )
            } else if (view.findViewById<EditText>(R.id.robot_access).text.toString() == "UGV") {
                findNavController().navigate(
                    R.id.action_ConnectFragment_to_clientFragment2,
                    mqttCredentialsBundle
                )
            } else if (view.findViewById<EditText>(R.id.robot_access).text.toString() == "RT") {
                findNavController().navigate(
                    R.id.action_ConnectFragment_to_clientFragmentRT2,
                    mqttCredentialsBundle
                )
            }
        }


                /*
            else if (view.findViewById<EditText>(R.id.robot_access).text.toString() == "cam") {
                findNavController().navigate(
                    R.id.action_ConnectFragment_to_camera,
                    mqttCredentialsBundle
                )
            }
            */

    }
}
