# Enable 3D Acceleration on Ubuntu 24.04 ARM in VMware

This guide assumes Ubuntu 24.04 ARM is already installed in a VMware Fusion or Workstation virtual machine.

## 1. Prepare the VM in VMware

1. Shut down the Ubuntu VM completely (do not suspend it).  
2. In VMware, open **Settings → Display** for the VM.  
3. Ensure **Accelerate 3D Graphics** is **unchecked** for now and close the settings window.  

## 2. Update Ubuntu and Install Open VM Tools

Inside Ubuntu, open a terminal and run:

```
sudo apt update
sudo apt full-upgrade -y
```

Install VMware guest tools integration:

```
sudo apt install -y open-vm-tools open-vm-tools-desktop
```

Reboot Ubuntu so the services load:

```
sudo reboot
```

## 3. Install Mesa Utilities and Vulkan Drivers

After the reboot, open a terminal again and run:

```
sudo apt install -y mesa-utils mesa-vulkan-drivers
```

These packages provide the Mesa OpenGL stack, the `glxinfo` utility, and Vulkan support used by VMware’s virtual GPU.

You can verify OpenGL later with:

```
glxinfo | grep "OpenGL"
```

Shut the VM down again:

```
sudo poweroff
```

## 4. Enable 3D Acceleration in VMware

1. With the VM powered off, open **Settings → Display** for the VM.  
2. Tick the **Accelerate 3D Graphics** checkbox.  
3. Optionally increase **Graphics Memory** if the UI allows it for better performance.  

Start the VM.

## 5. Confirm 3D Acceleration in Ubuntu

In Ubuntu, run:

```
glxinfo | grep -E "OpenGL (vendor|renderer|version)"
```

You should see a renderer associated with VMware’s virtual GPU instead of only the `llvmpipe` software renderer, indicating that 3D acceleration is active.

