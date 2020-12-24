# RPI 3 on Raspbian Buster Lite

### Download Image

[Download the Image](s3://cubosoft-private/IcedCoffeeRPI3Image.img) and flash it to the Pi's SD Card with Win32DiskImager.

### Manual PI setup

First set up Raspbian Buster Lite (see Raspberry PI docs for more info). Continue with the rest.

##### Set up SSH
In the boot drive, create an empty file `ssh` at the root level.

##### Set WiFi Password
IN the boot drive, at the root level create the file `wpa_supplicant.conf` with the following contents (edit wifi creds):

```
country=US # Your 2-digit country code
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
network={
    ssid="YOUR_NETWORK_NAME"
    psk="YOUR_PASSWORD"
    key_mgmt=WPA-PSK
}
```

##### Find out the PIs IP

Once powered up, the PI should connect to the WiFi automatically. Go to the routers IP address and look for connected devices (there's other ways, look it up).

##### Install Docker

SSH to the PI (my Pi's IP is 192.168.0.79):

```
 ssh pi@192.168.0.79
```

enter the password `raspberry` and change it (there's instructions printed once logged in).

Installing Docker is simple:

```
sudo apt update
sudo apt upgrade
curl -sSL https://get.docker.com | sh
sudo groupadd docker
sudo usermod -aG docker pi
```

log out and back in, now this should work:

```
docker ps
```

if it complains about docker `not found` (it did with me) rerun:

```
curl -sSL https://get.docker.com | sh
```
