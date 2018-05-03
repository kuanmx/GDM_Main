### Automated Welding Machine Controller Source Code

* Institution: University of Nottingham Malaysia Campus 
* Module: MMME4014 Group Design and Make 
* Project Title: Automated Welding Machine 
* Group Name: MMME4014-12 
* Item Description: Source Code for Controller

This source code is designed for develop with CLion with CMake. Tested on operating system: 
1. Fedora 28
2. Windows 10 Single Language
2. Windows 10 Professional

---
#### Setup Procedure (based on Fedora)
(if no C++ compiler) Install C++ compiler `yum install gcc-c++`
(optional) Install [Development Tools](https://www.2daygeek.com/install-development-tools-on-ubuntu-debian-arch-linux-mint-fedora-centos-rhel-opensuse/#)
                Group: `# yum groups install "Development Tools"`

1. Install following compiler: 
   * arm-none-eabi-g++  `dnf install arm-none-eabi-gcc-cs-c++`
   * arm-none-eabi-newlib `dnf install arm-none-eabi-newlib`
   
2. Install mbed-cli
   * Update Python  `pip install --upgrade pip`
   * Install mbed-cli via Python `pip install mbed-cli`
   * Install Mercurial `pip install mercurial`
   * Install missing package
   ```
   cd mbed-os
   pip install -r requirements.txt --user
   ```  
   
3. Cloning using mbed-cli
  * import using terminal at working directory `mbed import https://github.com/kuanmx/GDM_Main`
  
  OR
  * Deploy mbed-os after cloning `mbed deploy`
