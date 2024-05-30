Qwiic_OTOS_Py
===============

<p align="center">
   <img src="https://cdn.sparkfun.com/assets/custom_pages/2/7/2/qwiic-logo-registered.jpg"  width=200>  
   <img src="https://www.python.org/static/community_logos/python-logo-master-v3-TM.png"  width=240>   
</p>
<p align="center">
	<a href="https://pypi.org/project/sparkfun-qwiic-otos/" alt="Package">
		<img src="https://img.shields.io/pypi/pyversions/sparkfun_qwiic_otos.svg" /></a>
	<a href="https://github.com/sparkfun/Qwiic_OTOS_Py/issues" alt="Issues">
		<img src="https://img.shields.io/github/issues/sparkfun/Qwiic_OTOS_Py.svg" /></a>
	<a href="https://qwiic-otos-py.readthedocs.io/en/latest/?" alt="Documentation">
		<img src="https://readthedocs.org/projects/qwiic-otos-py/badge/?version=latest&style=flat" /></a>
	<a href="https://github.com/sparkfun/Qwiic_OTOS_Py/blob/master/LICENSE" alt="License">
		<img src="https://img.shields.io/badge/license-MIT-blue.svg" /></a>
	<a href="https://twitter.com/intent/follow?screen_name=sparkfun">
        	<img src="https://img.shields.io/twitter/follow/sparkfun.svg?style=social&logo=twitter"
           	 alt="follow on Twitter"></a>
	
</p>



<img src="https://cdn.sparkfun.com/assets/parts/2/5/2/0/9/SEN-24904-Optical-Tracking-Odometry-Sensor-Feature.jpg"  align="right" width=300 alt="SparkFun Optical Tracking Odometry Sensor - PAA5160E1 (Qwiic)">


Python module for the <a href="https://www.sparkfun.com/products/24904">SparkFun Optical Tracking Odometry Sensor - PAA5160E1 (Qwiic)</a>

This python package is a port of the existing <a href="https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library/">SparkFun Optical Tracking Odometry Sensor Arduino Library</a>.

This package can be used in conjunction with the overall <a href="https://github.com/sparkfun/Qwiic_Py">SparkFun qwiic Python Package.

New to qwiic? Take a look at the entire <a href="https://www.sparkfun.com/qwiic">SparkFun Qwiic ecosystem</a>.

### ⚠ **Using this sensor on a Raspberry Pi**? ⚠
Your system might need modification. See this <a href="#raspberry-pi-use">note</a>.

## Contents

* [Supported Platforms](#supported-platforms)
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Documentation](#documentation)
* [Example Use](#example-use)

Supported Platforms
--------------------
The qwiic otos Python package current supports the following platforms:
* [Raspberry Pi](https://www.sparkfun.com/search/results?term=raspberry+pi)
<!---
* [NVidia Jetson Nano](https://www.sparkfun.com/products/15297)
* [Google Coral Development Board](https://www.sparkfun.com/products/15318)
-->

Dependencies 
--------------
This driver package depends on the qwiic I2C driver: 
[Qwiic_I2C_Py](https://github.com/sparkfun/Qwiic_I2C_Py)

Documentation
-------------
The SparkFun qwiic otos module documentation is hosted at [ReadTheDocs](https://qwiic-otos-py.readthedocs.io/en/latest/?)

Installation
---------------
### PyPi Installation

This repository is hosted on PyPi as the [sparkfun-qwiic-otos](https://pypi.org/project/sparkfun-qwiic-otos/) package. On systems that support PyPi installation via pip, this library is installed using the following commands

For all users (note: the user must have sudo privileges):
```sh
sudo pip install sparkfun-qwiic-otos
```
For the current user:

```sh
pip install sparkfun-qwiic-otos
```
To install, make sure the setuptools package is installed on the system.

Direct installation at the command line:
```sh
python setup.py install
```

To build a package for use with pip:
```sh
python setup.py sdist
 ```
A package file is built and placed in a subdirectory called dist. This package file can be installed using pip.
```sh
cd dist
pip install sparkfun_qwiic_otos-<version>.tar.gz
```

Raspberry Pi Use
-------------------
For this sensor to work on the Raspberry Pi, I2C clock stretching must be enabled. 

To do this:
- Login as root to the target Raspberry Pi
- Open the file /boot/config.txt in your favorite editor (vi, nano ...etc)
- Scroll down until the block that contains the following is found:
```ini
dtparam=i2c_arm=on
dtparam=i2s=on
dtparam=spi=on
```
- Add the following line:
```ini
# Enable I2C clock stretching
dtparam=i2c_arm_baudrate=10000
```
- Save the file
- Reboot the raspberry pi

Example Use
 -------------
See the examples directory for more detailed use examples.

```python
from __future__ import print_function
import qwiic_otos
import sys
import time

def runExample():
	print("\nQwiic otos Example 1 - Buzz\n")

	# Create instance of device
	my_otos = qwiic_otos.Qwiicotos()

	# Initialize the device
	if my_otos.begin() == False:
		print("The device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	print("\nQwiic otos ready!")
	
	# Loop forever
	while True:
		my_otos.on()
		time.sleep(1)
		my_otos.off()
		time.sleep(1)     

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example")
		sys.exit(0)

```
<p align="center">
<img src="https://cdn.sparkfun.com/assets/custom_pages/3/3/4/dark-logo-red-flame.png" alt="SparkFun - Start Something">
</p>
