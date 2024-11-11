# Setting up an Ubuntu 24 VM on Windows

This tutorial is meant for Windows users that are following this robotics course and who do not have a separate, 
system-wide installation of Ubuntu 24 or any other Linux distro readily available.


## Installing Oracle Virtualbox

1. Download Oracle VirtualBox by following [this](https://www.virtualbox.org/wiki/Downloads) link and clicking the 
"Windows hosts" option in the left menu.

2. Once the download is completed, install Oracle VirtualBox by following the on-screen prompts.

During the "Warning: Network Interfaces" step of the installation, choose "Yes".

If the "Missing Dependencies - Python Core / win32api" step appears during your installation, also choose "Yes".


## Creating your custom Ubuntu 24 VM

1. Download the official Ubuntu 24 `.iso` image from [this](https://ubuntu.com/download/desktop) link. Make sure to 
download version `24.04.1 LTS`

2. Once the `.iso` image downloaded, start VirtualBox and follow these steps:

- Create a new VM by pressing "New", in the main VirtualBox view.

<img src="/0-setup/images/create_ubuntu_vm_step_1.png" alt="Create Ubuntu VM step 1">

- On the next step, you need to configure the first details about your VM. Add a descriptive name, choose a new folder 
on your machine on a disk where you have plenty of space, as this is the folder where your VM will be installed and where
all the files on the VM will be hosted. Finally, add the path of the Ubuntu 24 `.iso` image that you downloaded, and make 
sure that you tick the "Skip Unattended Installation" box (this will allow you to go through all the steps of installing
Ubuntu 24 inside your VM) This is how the VirtualBox window should look before you click "Next":

<img src="/0-setup/images/create_ubuntu_vm_step_2.png" alt="Create Ubuntu VM step 2">

- On this step we will configure the base memory and number of CPU cores that your VM is allowed to use. You should base
the values you put here on the specifications of your computer, so make sure you don't put too much. In this example we 
are going with 8192 Mb of RAM (8 Gb) and 6 CPU cores. We recommend at least 4 Gb of RAM and 4 CPU cores. You can also go
with as low as 2 Gb of RAM, but the performance of your VM along with our demos will take a hit. The VirtualBox windows should look something like this 
before you click "Next":

<img src="/0-setup/images/create_ubuntu_vm_step_3.png" alt="Create Ubuntu VM step 3">

- On this step we are allocating the maximum amount of hard disk space for our VM. We are going with 80 Gb. You can put 
even more if you would like, or less, but we would not recommend less than 40 Gb. This is how VirtualBox should look like 
before you click "Next":

<img src="/0-setup/images/create_ubuntu_vm_step_4.png" alt="Create Ubuntu VM step 4">

- On this step check again that all the details of your new VM look good and then click "Finish".


## Installing Ubuntu 24 inside the VM
### Prerequisite
Before we install Ubuntu 24, we need to do some extra configuration to make sure that our VM can access the internet 
from our host machine (Windows). For this, we need to make sure that the network settings on our VM are suitable:

- From the main view of VirtualBox, click your new VM and then click "Settings".

<img src="/0-setup/images/configure_network_1.png" alt="Configure VM network step 1">

- From the "Basic" VM settings section, check that in the "Network" section, "Adapter 1" is enabled and that it is 
attached to "NAT". Click "Ok" after checking.

<img src="/0-setup/images/configure_network_2.png" alt="Configure VM network step 2">

- On Windows 11, go to "Advanced Network Settings" and under "Network Adapters" make sure to enable the 
"VirtualBox Host-Only Ethernet Adapter". This is how the window should look like. For Windows 10 similar steps should be
followed.

<img src="/0-setup/images/configure_network_3.png" alt="Configure VM network step 3">

### Installing Ubuntu 24
- On the main view of VirtualBox, choose the VM you just created and click "Start" (alternatively you can double-click
the selected VM) to run it for the first time. We will now go through the process of installing Ubuntu inside the VM.

<img src="/0-setup/images/install_ubuntu_1.png" alt="Install Ubuntu step 1">

- Once the VM boots up for the first time, you will be greeted by a Grub menu. Click inside VM and choose the "Try or 
install Ubuntu" option by pressing Enter.

<img src="/0-setup/images/install_ubuntu_2.png" alt="Install Ubuntu step 2">

- Once the Ubuntu preview boots up, you should be greeted by a menu walking you through the steps of installing Ubuntu.
  - Choose your preferred language.
  - Choose your accessibility settings. If you require no special settings, just press "Next".
  - Select your keyboard layout.
  - On the "Connect to the internet" step, if you followed the prerequisite steps from this section, you should select 
the "Use wired connection" option.
  - On the next step choose "Install Ubuntu".
  - Choose "Interactive installation".
  - For the app selection step you can go with the "Default selection".
  - On the "Install recommended proprietary software" step, tick both boxes before proceeding.
  - On the next step choose "Erase disk and install Ubuntu". This won't have any special effect as Ubuntu 24 will be 
installed to the new folder you created and chose while creating the VM.
  - On the next step, choose the details of your main user account for Ubuntu. You can also choose a password if you wish.
The window should look something like this:
  <img src="/0-setup/images/install_ubuntu_3.png" alt="Install Ubuntu step 3">
  - Select your timezone.
  - Review your choices and click "Install".
  - Wait for Ubuntu 24 to finish installing inside your VM. Click "Restart now" once the installation finishes.
  - On reboot, you will be greeted by a screen prompting you to remove the installation medium. Just click on the screen
and then press Enter.
  - Once Ubuntu 24 boots up, log into the user you created (input the password if you chose one). During the first boot 
wizard skip Ubuntu Pro, opt out of sharing system data, and then click "Finish". You now have successfully installed 
Ubuntu 24 inside your new VM!


## Configuring the VM

### Check if the internet access is working correctly
Inside your VM, open Firefox or any other browser and try accessing any website. If you Windows host machine has Internet
access, then you should also be able to access the internet from inside the VM.

### Increase VM video memory and enable 3D acceleration
VirtualBox VMs do not have access to your host machine's dedicated GPU, and there is no way for you to enable GPU 
passthrough for your VM. However, the VM has an emulated GPU that uses video memory allocated from your host machine's
RAM. 

Let's increase the available video memory for the VM and enable the 3D acceleration. To do this, with the VM powered off
(just close the window where the VM is operating and VirtualBox will ask you to power off the VM), go again to the 
"Settings" of the VM, switch to the "Expert" tab, and under "Display" increase the "Video Memory" to 128 MB and tick the 
"Enable 3D Acceleration" box. Then click "Ok".

<img src="/0-setup/images/enable_3D_acceleration.png" alt="Enable 3D acceleration">

### Enable the shared clipboard and optionally Drag'n'Drop functionality between your host machine and the Ubuntu 24 VM
Sometimes it can be quite a hassle to copy information and files between your Windows host machine and the Ubuntu VM.

To enable both the shared clipboard and also the Drag'n'Drop functionality, go to the basic settings of the VM when it is 
powered off, select "General" and from the "Advanced" tab of the "General" section, enable both of these 
functionalities in the way you see fit!

<img src="/0-setup/images/clipboard.png" alt="clipboard">

### Enable custom screen resolutions for your VM
You might have already observed that the screen resolution of the Ubuntu 24 VM is quite small, and sometimes it doesn't 
scale nicely with the size of your actual screen. We can enable custom resolutions for our VM by installing the 
"VirtualBox guest additions" inside Ubuntu. 

1. First we need to install some required packages. In Ubuntu open a new terminal (Ctrl + Alt + t or right-click on Desktop
and click "Open in Terminal"). Run the following commands inside the new terminal:

```commandline
    sudo apt update
    sudo apt install build-essential linux-headers-$(uname -r) -y
```

2. We then mount the "Guest Additions" iso image by going to "Devices" -> "Insert Guest Additions CD image..." in the 
VirtualBox tab where our Ubuntu VM is running.

3. Once you do this, you will see in the left menu inside Ubuntu a small CD icon pops up. Click it. Open a new terminal 
inside the window that pops up.

<img src="/0-setup/images/guest_additions.png" alt="Guest additions 1">

4. In the opened terminal run:

```commandline
    sudo ./VBoxLinuxAdditions.run
```

5. After the installation completes, reboot the VM by running:

```commandline
    sudo reboot
```

You can now change the resolution and the scale factor of the Ubuntu 24 VM by going to "View" -> "Virtual Screen 1" -> 
choose resolution and scale factor. This, coupled with the option from "View" -> "Auto-resize Guest Display" will make 
your experience of working with your new VM more seamless!


## Congratulations! You have now successfully installed and configured your own Ubuntu 24 VM!
