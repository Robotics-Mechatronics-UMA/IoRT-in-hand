package com.example.mqttkotlinsample

import android.content.Context
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.os.Build
import android.os.Bundle
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import android.widget.Toast
import android.widget.Toast.*
import androidx.annotation.RequiresApi

class MainActivity : AppCompatActivity() {

    @RequiresApi(Build.VERSION_CODES.M)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        setSupportActionBar(findViewById(R.id.toolbar))

        /* Check if Internet connection is available */
        if (!isConnected()) {
            Log.d(this.javaClass.name, "Internet connection NOT available")

            makeText(applicationContext, "Internet connection NOT available", LENGTH_LONG).show()
        }
    }

    override fun onStart() {
        super.onStart()

        //val mytoast = Toast.makeText(this, "onStart is called", Toast.LENGTH_SHORT)
        //mytoast.show()
    }
/*
    override fun onResume() {
        super.onResume()
        val mytoast = Toast.makeText(this, "onResume is called", Toast.LENGTH_SHORT)
        mytoast.show()    }

    override fun onPause() {
        super.onPause()
        val mytoast = Toast.makeText(this, "onPause is called", Toast.LENGTH_SHORT)
        mytoast.show()
    }

    override fun onStop() {
        super.onStop()
        val mytoast = Toast.makeText(this, "onStop is called", Toast.LENGTH_SHORT)
        mytoast.show()    }
*/
    private fun isConnected(): Boolean {
        var result = false
        val cm = getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            val capabilities = cm.getNetworkCapabilities(cm.activeNetwork)
            if (capabilities != null) {
                result = when {
                    capabilities.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) ||
                            capabilities.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) ||
                            capabilities.hasTransport(NetworkCapabilities.TRANSPORT_VPN) -> true
                    else -> false
                }
            }
        } else {
            val activeNetwork = cm.activeNetworkInfo
            if (activeNetwork != null) {
                // connected to the internet
                result = when (activeNetwork.type) {
                    ConnectivityManager.TYPE_WIFI,
                    ConnectivityManager.TYPE_MOBILE,
                    ConnectivityManager.TYPE_VPN -> true
                    else -> false
                }
            }
        }
        return result
    }
}