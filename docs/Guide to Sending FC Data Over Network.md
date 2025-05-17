# Guide to Sending Flight Controller Data Over Network

This guide covers how to wirelessly access flight controller data on a pc being sent from a laptop.
Both devices use Windows 10

## Prerequisites
* Two devices with MissionPlanner and MAVProxy installer
* Router
* Flight controller

## Setup
### Set Up the Hardware
1. Connect socket and plug into wall
2. Plug usb-c power cable into router
3. Plug the ethernet cable into the router (port labelled LAN) and the **laptop**
4. Plug the flight controller into the **laptop**

Setup should look like this:

<img src="../assets/hardware setup.jpg" alt="hardware setup" width="50%">

### Find Port Name of Flight Controller
1. Go to device manager
2. Scroll down to "Ports (COM & LPT)"
3. Find the port name for the flight controller (It might help unplugging and replugging in the flight controller, and seeing what port name appears)\
    Device manager before plugging in fc:\
    ![Ports before fc](<../assets/device manager 1.PNG>)\
    Device manager after plugging in fc:\
    ![Ports after fc](<../assets/device manager 2.PNG>)\

4. Keep note of the relevant port name for later

### Get the PC's ip address on the network
1. On the **PC** go to wifi and connect to "GL-SFT1200-502" (this should be on the back of the router)
2. Input the password `goodlife` (this should also be on the back of the router)

    ![goodlife](<../assets/goodlife.PNG>)\

3. On the **laptop** enter `192.168.8.1` into your browser
4. Enter the password `Marsworks12345!`
5. On the dashboard, click "CLIENTS" to the left. You should see the ip addresses of the two devices

    ![dashboard clients](<../assets/gl clients.PNG>)\

6. Keep note of the **laptop's** ip address for later

### Create a UDP Firewall Exception
1. Go to "Windows Defender Firewall with Advanced Security”
2. Click "Outbound rules" then "New rule"

    ![firewall](<../assets/firewall.png>)\

3. Select "Port" and click "Next"
4. Select "UDP", type `14550` in "Specific remote ports:" and click "Next"
5. Select "Allow the connection" and click "Next"
6. Tick all rules and click "Next"
7. Name this rule and click "Finish"

## Sending and Receiving Data
1. On your **laptop** open your termainal as an **administrator**
2. Type in: `mavproxy.exe --master=[port] --baudrate=115200 --out=udp:[pc_ip]:14550`
    Where:
    * [port] is the port name you saved from earlier (eg/ "COM7")
    * [pc_ip] is the ip address of the device connected wirelessly to the router

    (An example of this command might look like: `mavproxy.exe --master=com7 --baudrate=115200 --out=udp:123.456.7.890:14550`)
3. Open MissionPlanner on your **PC**
4. In the top right corner, select "UDP" from the first drop-down
5. Type `115200` in the box next to this and click "Connect"
6. Type `14550` in the text box that says "Enter Local port"

You should now be able to see the flight controller data on your device which is connected wirelessly to the network

