//////////////////////////////
EXPLANATION:
The "index.html" file in the "ESP" folder is the custom remote controller website you can host on whatever device you have the file in (server,phone,laptop,etc.) 

WARNING, this has only been tested on a laptop w. python

Also do not remove it from the "ESP" folder

"ESP32GroupProject.ino" is the Arduino IDE code for the ESP32 Microcontroller. 

//////////////////////////////
HOW-TO-RUN:
1.Download / Un-Zip the files.
2.Ensure Ardunio IDE is setup for the ESP32-S3 microcontroller
3.Upload to the ESP32-S3 via the Ardunio IDE
4.Ensure Python is installed
5.In Terminal setup the directory to the file (EXAMPLE:  cd "C:\Users\LoganSmith\OneDrive\Desktop\ESP")
6.In Terminal launch a python server to the directory (EXAMPLE: python -m http.server 8000)
7.Go to google chrome or windows browser to the website http://localhost:8000/

You should now see the controller, as long as the hardware is setup correctly, you can connect via BLE to the esp and control the arm.