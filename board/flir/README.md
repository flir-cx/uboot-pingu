SUMMARY
=======

This is the documenation of u-boot for flir cameras.

sandbox
-------

To run sanbox tests in general see the follwing links:

* <https://u-boot.readthedocs.io/en/latest/develop/tests_sandbox.html>
* <https://u-boot.readthedocs.io/en/v2021.04/develop/py_testing.html>

Sandbox is a u-boot architecture that runs on your local pc and compiled
with the native tools. So to run it you should **NOT** source the tool chain you use to build for your target.

Steps to get it up and running for the first time:

```bash
# Install som dependencies:
sudo apt install libsdl2-dev libgnutls28-dev libgit2-dev python3 python3-virtualenv
# Checkout the u-boot code
git clone -b FLIR_EC101_v21_04 ssh://git@bitbucketcommercial.flir.com:7999/camos/uboot-pingu.git
cd uboot-pingu
# Create a virtual environment for python
virtualenv -p /usr/bin/python3 venv
# Start venv. This is needed every time you start a new terminal to build from.
. ./venv/bin/activate
# Install required python packages
pip install -r test/py/requirements.txt
# Build the sandbox configuration
make O=build-sandbox flir_sandbox_defconfig
# Take the sandbox out for a test spin
make O=build-sandbox -j16
./build-sandbox/u-boot
```

To run the pyhon tests
`./test/py/test.py --bd sandbox`

If you want to test for example buttons you can start sandbox like this:
`./build-sandbox/u-boot -d build-sandbox/u-boot.dtb`
If you then in u-boot do:
`button list`
according to sanbox.dtsi button1 is connected to gpio a3 and button2 is connected to gpio a4. So if you try:
```
button button1
gpio toggle a3
button button1
```
you will see that the buttons are actually working as expected.