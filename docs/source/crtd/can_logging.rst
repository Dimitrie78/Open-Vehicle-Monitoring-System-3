.. highlight:: none

====================
CAN Bus Data Logging
====================

OVMS can be used as CAN bus datalogging tool.


--------------------
Physical Connections
--------------------

OVMS hardware V3 supports up to three CAN bus connections. The connections to OVMS are as follows:

::

  DB9-F   Signal
  -----   ------
    2     CAN1-L
    7     CAN1-H
  
    4     CAN2-L
    5     CAN2-H
    
    6     CAN3-L
    8     CAN3-H

Note: the board schematics refer to CAN0,1,2.  These correspond to CAN1,2,3 here in the documentation and source code.  The first is handled by ESP32 i/o lines directly, and CAN2/3 are handled by MCP2515 ICs via SPI from the ESP.

CAN1 is the fastest bus, use this one if possible. The CAN logging tool is able to log all buses at the same time, to the same file or stream.

Vehicle CAN bus(s) are usually accesable via the vehicle's OBD2 port. Most modern cars have multiple CAN busses. The OBD2 'standard' CAN will be available on OBD2 **pin 6: CAN-H** and **pin 14: CAN-L**. However, modern vehicles (especially EV's) often have other CAN buses available on non-standard OBD2 pins.

A voltmeter (ideally oscilloscope) can be used to determine which OBD2 pins contain CAN data:

* Can high pins should normally be between 2.5 and 3.5 volts (to ground) - maybe 2.7 to 3.3 volts if there is traffic.
* Can low pins should normally be between 1.5 and 2.5 volts (to ground) - maybe 1.7 to 2.3 volts if there is traffic.

Pre-fabricated OBD2 > DB9-F for several specific vehicles can be purchased via OpenVehicles.


-------------------
Enable OVMS CAN bus
-------------------

Once physical connections has been made and OVMS is up and running connect to OVMS shell via web browser / SSH or serial.

If a specific vehicle module is loaded the CAN bus will already be enabled in OVMS. To check which CAN buses are enabled use:

``OVMS# can list``
  
If no vehicle module is selected the CAN bus must be started e.g

``OVMS# can can1 start listen 500000``
  
This will enable ``CAN1`` in ``listen`` mode (read only) at 500k baud, ``active`` can be used instead of listen to enable read-write mode. To stop a ``CAN1`` bus:

``OVMS# can can1 stop``
  
OVMS supports the following CAN bauds rates: ``100000, 125000, 250000, 500000, 1000000``.


------------------
Logging to SD card
------------------

It is possible to view CAN data directly in OVMS monitor shell, however since modern cars have very busy CAN buses there is often too much data which swamps the monitor or exceeds the logging queue, resulting in dropped messages. Logging to SD card is the better option.

If using a good quality SD card with a current OVMS V3 module (i.e. PCB revision 3.2 / 2019.05.23 or later), increase the SD card speed for best performance with:

``config set sdcard maxfreq.khz 20000``

.. warning::
  If you increase :code:`maxfreq.khz` too much, higher than the maximum possible frequency supported by the board,
  you may encounter a bootloop on the next boot.
  In that case you will want to eject the SD card - to stop the bootloop - and change the config to a lower value of :code:`maxfreq.khz`.

Start logging all CAN messages using CRTD log file format with:

``ovms# can log start vfs crtd /sd/can.crtd``
  
or log specific CAN packets by applying a filter e.g 0x55b the Nissan LEAF SoC CAN message

``ovms# can log start vfs crtd /sd/can.crtd 55b``
  
Other CAN log file formats are supported e.g ``crtd, cs11, gvret-a, gvret-b, lawicel, pcap, raw``.
  
Check CAN logging satus with:

``ovms# can log status``

To Stop CAN logging:

``ovms# can log stop``

**Note: the can logging must be stopped before the file can be viewed**

To View the CAN log:

``ovms# vfs head /sd/can.crtd``
  
``tail`` and ``cat`` commands can also be used. However, be careful the log file can quickly become very large, ``cat`` may overwhelm the shell.

The log file can also be viewed in a browser with ``http://<ovms-ipaddress>/sd/can.crtd``
  

The logfiles can then be imported into a tool like SavvyCan for analysis.


--------------------------
Logging Events and Metrics
--------------------------

Alongside the CAN data, it's also possible to log any **event** or **metric** of your choosing.

For an event, the name of the event will be logged. For a metric, a JSON representation of the metric will be logged
(an object with 3 properties: ``name``, ``value``, and ``unit``).

To select the events and/or metrics to log, a comma-separated list of filter (``"<filter1>,<filter2>,..."``) needs to be
configured:

* events filters are configured by the configuration item ``can log.events_filters``
* metrics filters are configured by the configuration item ``can log.metrics_filters``

Each filter of the list can be one of:

a) an event or metric name that will be matched in its entirety (e.g. matching the metric ``v.e.charging12v``, or the event ``system.wifi.ap.sta.connected``)
b) a pattern ending with a wildcard ``*`` - to match the beginning of an event or metric name (e.g. ``v.p.*`` will match all metrics starting with ``v.p.``, like ``v.p.odometer`` for instance)
c) a pattern starting with a wildcard ``*`` - to match the end of an event or metric name (e.g. ``*.stop`` will match all metrics ending with ``.stop``, like ``network.mgr.stop``, ``system.wifi.ap.stop``, ... for instance)

.. note:: Only those 3 kind of filters are supported. The wildcard character ``*`` can only occur once, either at the beginning, or
  at the end of the filter.

^^^^^^^^^^^^^^^
Default filters
^^^^^^^^^^^^^^^

The default filter configuration for **event** logging is to log all events starting with ``x`` or with ``vehicle``, which is equivalent to the following configuration command::

  OVMS# config set can log.events_filters "x*,vehicle*"


The default filter configuration for **metric** logging is not to log any metric.

^^^^^^^^
Examples
^^^^^^^^

If you would like to log all GNSS **events** for example, in addition to the default events, you could use the following configuration::

  OVMS# config set can log.events_filters "x*,vehicle*,gps.*"


If you would like to log all GNSS **metrics** for example, you could use the following configuration::

  OVMS# config set can log.metrics_filters "v.p.gps*, v.p.latitude, v.p.longitude, v.p.altitude, v.p.direction, v.p.satcount"

^^^^^^^^^^^^^^^^^^^^^
Supported log formats
^^^^^^^^^^^^^^^^^^^^^
For the moment, only the CRTD log format is able to store the events or metrics in the logs.
Those are logged with the tags:

* ``CEV`` for an event
* ``CMT`` for a metric

Example of a CRTD log output containing a mix of CAN messages, metrics and events::

  1668992145.032123 1CMT Metric { "name": "v.p.satcount", "value": 6, "unit": "" }
  1668992145.034551 1CMT Metric { "name": "v.p.gpshdop", "value": 1.1, "unit": "" }
  1668992145.036341 1R11 358 18 08 20 00 00 00 00 20
  1668992145.037777 1CEV Event vehicle.alert.tpms
  1668992145.041696 1CMT Metric { "name": "v.p.altitude", "value": 121.3, "unit": "m" }
  1668992147.042809 1R11 27E c0 c0 c0 c0 00 00 00 00
  1668992150.035591 1CMT Metric { "name": "v.p.gpssq", "value": 20, "unit": "%" }
  1668992150.042837 1CEV Event gps.sq.bad

-----------------
Network Streaming
-----------------

CAN data can be streamed directly to SavvyCan (or other compatible application) using the OVMS tcpserver CAN logging feature over a local network. Start tcpserver CAN logging with:

``OVMS# can log start tcpserver discard gvret-b :23``

This will start a tcpserver on port 23 (as required by SavvyCan) using the GVRET format supported by SavvyCAN. 

Once OVMS CAN logging tcpserver is running open up SavvyCan and select: 

``Connection > Add New Device Connection > Network Connection`` 

then enter the OVMS WiFi local network IP address (no port number required). CAN packets should now appear streaming into SavvyCan. 

*Note: CAN tcpserver network streaming is a beta feture currently in edge firmware and may be buggy*


--------------------------
Optimizing the Performance
--------------------------

On ``can log stop``, the system will output some statistics. Check especially the dropped frame count.
Frame drops can occur because the system was busy with other tasks like handling network traffic. 
There are two options to optimize this:

a) Reduce background activities, i.e. stop all services not needed for the logging.
   If possible, do the logging without an active vehicle module (e.g. set the 
   "empty" vehicle via ``vehicle module NONE``).

b) Raise the log queue size. The default queue size has a capacity of 100 frames.
   To e.g. allow 200 frames, do: ``config set can log.queuesize 200``.

