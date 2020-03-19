# Accessing GPIOs on ODROID XU4

[RPi.GPIO webside](https://wiki.odroid.com/odroid-xu4/application_note/gpio/rpi.gpio)  

GPIO on odroid can be accessed through python 2 by using a raspberry pi package called [wiringpi](http://wiringpi.com/reference/core-functions/):

```
git clone https://github.com/hardkernel/wiringPi
cd wiringPi
sudo ./build
```

and a wrapper liberary that ports it to odroid called [wiringpi](https://wiki.odroid.com/odroid-n2/application_note/gpio/wiringpi#tab__github_repository)): 

```
sudo python -m pip install odroid-wiringpi
```


A reference of core functions can be found on the [wiringpi website](http://wiringpi.com/reference/core-functions/) and the mapping between ids and PINs can be found on the [odroid wiki](https://wiki.odroid.com/odroid-xu4/software/gpio_register_map).