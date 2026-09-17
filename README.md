# ForkliftVibration

## Architecture

The application uses three containers:

- MariaDB stores sessions, GPS fixes, shock events, and exposure summaries.
- The Python container supervises the ADXL343 acquisition process, GPS reader,
	LCD updater, and Flask API. Acquisition writes through a queued database
	writer so SD-card latency does not block the accelerometer FIFO.
- Nginx serves the dashboard and reverse-proxies `/api/` to Flask. Keep Nginx
	as the only published application port.

The SQL schema is designed for this split. Waveforms remain in `/data/wbv_events`
and the database stores their path and SHA-256; the CSV export contains the
shock value and GPS coordinates needed for mapping.

## Raspberry Pi prerequisites

Enable I2C and the hardware serial port. Disable the serial login shell. The
Compose file expects `/dev/i2c-1`, `/dev/serial0`, `/dev/gpiomem`, and
`/dev/gpiochip0`. Connect ADXL343 INT1 to GPIO17 if interrupt mode is later
enabled; the default uses FIFO polling for simpler container permissions.

Create a `.env` beside `compose.yaml` before starting:

```dotenv
DB_ROOT_PASSWORD=change-this-root-password
DB_NAME=wbv
DB_USER=wbv_app
DB_PASSWORD=change-this-app-password
```

Create the HTTPS login file before starting Nginx:

```sh
sudo apt update
sudo apt install apache2-utils
mkdir -p auth
htpasswd -c auth/.htpasswd operator
```

Keep `auth/.htpasswd` outside `html/`; Nginx mounts it only at
`/etc/nginx/auth/.htpasswd` and it is never served as website content.

The first MariaDB initialization runs `SQL/Database_schema.sql`. Use a USB SSD
for the MariaDB volume on a production forklift logger; continuous writes to an
SD card shorten its life. A UPS or supercapacitor HAT is also recommended so
the session can close cleanly during forklift power loss.

Start and stop the complete application with:

```sh
docker compose up -d --build
docker compose down
```

The dashboard is served at `https://<pi-hostname>/`. The CSV endpoint is
`/api/events.csv`; add `?date=YYYY-MM-DD` to export one UTC day.

## Private Wi-Fi access point

The Pi can provide a private Wi-Fi network for a nearby computer. This uses
NetworkManager's built-in access-point and DHCP support; it does not bridge or
bridge the Wi-Fi network to another network. NetworkManager may provide NAT
through another active connection in shared mode.

Check that the selected adapter supports AP mode before using it:

```sh
iw list | sed -n '/Supported interface modes:/,/Band /p'
```

On the current Pi, the built-in `wlan0` supports AP mode, but the Edimax
`wlan1` adapter does not. Therefore the `wlan1` setup script cannot work with
that adapter. Use the built-in radio instead:

```sh
sudo nmcli device wifi hotspot ifname wlan0 \
	con-name forklift-wlan0-ap \
	ssid ForkliftTracker \
	password 'replace-with-a-strong-password'
```

This may disconnect `wlan0` from its current Wi-Fi network. A separate USB
adapter that advertises `AP` mode can be used with `setup-wlan1-ap.sh`.

Install NetworkManager if needed, then run the setup script with an SSID and a
WPA2 password of at least eight characters:

```sh
sudo apt update
sudo apt install network-manager
sudo systemctl enable --now NetworkManager
sudo ./setup-wlan1-ap.sh ForkliftTracker 'replace-with-a-strong-password'

```

Connect the computer to that SSID. NetworkManager assigns it an address in
`192.168.50.0/24`, and the Pi is `192.168.50.1`. Open
`https://192.168.50.1/` and sign in with the Nginx login. Because the existing
certificate is not issued for the IP address, a browser certificate warning is
expected unless the certificate is regenerated for the chosen hostname.

To stop the access point:

```sh
sudo nmcli connection down forklift-wlan1-ap
```

The setup script replaces only the connection named `forklift-wlan1-ap`; it
does not change other Wi-Fi connections.

## Measurement note

The logger applies Wk weighting to the vertical axis and Wd weighting to the
horizontal axes, records peak shock, crest factor, VDV contribution, clipping,
and interpolated GPS position. ISO 2631-1 exposure values depend on correct
mounting, calibration, seat position, axis orientation, and the actual daily
exposure time. Validate the physical installation and weighting coefficients
against the licensed standard before using results for compliance decisions.

## Improving GPS accuracy

The GPS HAT is enabled for SBAS/WAAS corrections and reports fixes at 5 Hz.
For the best results, mount its antenna horizontally with a clear view of the
sky, away from the forklift's motor, radio transmitters, metal obstructions,
and high-current wiring. Keep the antenna ground plane level and use a good
power supply and short, secure serial wiring.

The dashboard records `hdop`, satellite count, and position source for each
event. A clear-sky MTK3339 fix is normally accurate to a few metres; GPS
interpolation improves event timing but cannot create accuracy the receiver did
not measure. For sub-metre or repeatable lane-level coordinates, use an
external multi-band RTK receiver with a surveyed base or correction service.
