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
