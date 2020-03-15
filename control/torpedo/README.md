# Accessing GPIOs on ODROID XU4

GPIO on odroid can be accessed through python 2 by using a wrapper library (that is hardforked from a raspberry pi package) called [odroid-wiringpi](https://wiki.odroid.com/odroid-n2/application_note/gpio/wiringpi#tab__github_repository). It can be installed using pip:

```sudo python -m pip install odroid-wiringpi```

A reference of its core functions can be found on the [wiringpi website](http://wiringpi.com/reference/core-functions/).

The mapping between ids and PINs can be found on the [odroid wiki](https://wiki.odroid.com/odroid-xu4/software/gpio_register_map).