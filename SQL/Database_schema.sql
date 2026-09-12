-- Forklift whole-body vibration / shock logging schema
-- MySQL 8.0+ / MariaDB 10.4+ (Raspberry Pi OS ships MariaDB).
-- Rename anything; the Python layer takes table names from config.
--
--   sudo mysql < schema.sql
--
-- Indexes are declared inline rather than as CREATE INDEX IF NOT EXISTS,
-- which MariaDB supports but MySQL 8 does not.

CREATE DATABASE IF NOT EXISTS wbv
  CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
USE wbv;

-- ---------------------------------------------------------------------
-- Fleet registry
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS truck (
    truck_id      INT AUTO_INCREMENT PRIMARY KEY,
    asset_tag     VARCHAR(64)  NOT NULL,
    make_model    VARCHAR(128),
    seat_type     VARCHAR(64),          -- suspension or static: changes everything
    tyre_type     VARCHAR(64),          -- cushion vs pneumatic
    notes         TEXT,
    created_at    DATETIME(3)  NOT NULL DEFAULT CURRENT_TIMESTAMP(3),
    UNIQUE KEY truck_asset_uq (asset_tag)
) ENGINE=InnoDB;

CREATE TABLE IF NOT EXISTS logger (
    logger_id      INT AUTO_INCREMENT PRIMARY KEY,
    hostname       VARCHAR(128) NOT NULL,
    pi_serial      VARCHAR(32),
    accel_part     VARCHAR(32)  NOT NULL DEFAULT 'ADXL343',
    mount_location VARCHAR(64),         -- 'seat pad' or 'chassis'
    created_at     DATETIME(3)  NOT NULL DEFAULT CURRENT_TIMESTAMP(3),
    UNIQUE KEY logger_host_uq (hostname)
) ENGINE=InnoDB;

-- ---------------------------------------------------------------------
-- One row per logger run. Holds everything needed to reinterpret the
-- numbers later: change the weighting or the trigger threshold and old
-- rows must not be silently compared against new ones.
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS session (
    session_id          BIGINT AUTO_INCREMENT PRIMARY KEY,
    session_uuid        CHAR(36)     NOT NULL,
    logger_id           INT,
    truck_id            INT,
    operator_ref        VARCHAR(64),           -- pseudonymous if you need it
    started_at          DATETIME(6)  NOT NULL, -- UTC
    ended_at            DATETIME(6),

    sample_rate_hz      DOUBLE       NOT NULL,
    accel_range_g       SMALLINT     NOT NULL,
    fifo_watermark      SMALLINT,
    shock_threshold_ms2 DOUBLE       NOT NULL,
    pre_trigger_s       DOUBLE       NOT NULL,
    post_trigger_s      DOUBLE       NOT NULL,
    -- bump this when the weighting coefficients change, or old and new
    -- a_w values silently stop being comparable
    weighting_version   VARCHAR(32)  NOT NULL,
    k_factor_x          DOUBLE       NOT NULL DEFAULT 1.4,
    k_factor_y          DOUBLE       NOT NULL DEFAULT 1.4,
    k_factor_z          DOUBLE       NOT NULL DEFAULT 1.0,
    gating_enabled      TINYINT(1)   NOT NULL DEFAULT 1,
    config              JSON,                  -- full argument dump
    created_at          DATETIME(3)  NOT NULL DEFAULT CURRENT_TIMESTAMP(3),

    UNIQUE KEY session_uuid_uq (session_uuid),
    KEY session_started_idx (started_at),
    KEY session_truck_idx (truck_id, started_at),
    CONSTRAINT session_logger_fk FOREIGN KEY (logger_id)
        REFERENCES logger(logger_id),
    CONSTRAINT session_truck_fk FOREIGN KEY (truck_id)
        REFERENCES truck(truck_id)
) ENGINE=InnoDB;

-- ---------------------------------------------------------------------
-- GPS track, kept separately from events so positions can be re-derived
-- if the interpolation changes, and so you can draw the route.
-- At 5 Hz this is ~18k rows/hour. See the retention note at the bottom.
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS gps_fix (
    fix_id      BIGINT AUTO_INCREMENT PRIMARY KEY,
    session_id  BIGINT      NOT NULL,
    fix_time    DATETIME(6) NOT NULL,      -- UTC, from the RMC sentence
    lat         DOUBLE      NOT NULL,
    lon         DOUBLE      NOT NULL,
    altitude_m  DOUBLE,
    speed_mps   DOUBLE,
    course_deg  DOUBLE,
    fix_quality TINYINT,                   -- GGA field: 0 none, 1 GPS, 2 DGPS
    satellites  TINYINT,
    hdop        DOUBLE,

    UNIQUE KEY gps_fix_uq (session_id, fix_time),
    CONSTRAINT gps_fix_session_fk FOREIGN KEY (session_id)
        REFERENCES session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ---------------------------------------------------------------------
-- Shock events: one row per triggered capture.
--
-- Waveforms stay on disk. A 3 s capture at 800 Hz x 6 channels is ~60 kB
-- compressed; at a few hundred events a shift that is real write volume,
-- and putting it in InnoDB on an SD card is how you destroy the card.
-- waveform_path points at the .npz; waveform_sha256 detects bit rot.
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS shock_event (
    event_id       BIGINT AUTO_INCREMENT PRIMARY KEY,
    session_id     BIGINT      NOT NULL,
    event_time     DATETIME(6) NOT NULL,   -- trigger sample, UTC
    seq            INT         NOT NULL,   -- per-session counter

    peak_ms2       DOUBLE      NOT NULL,   -- weighted and k-scaled
    axis           ENUM('x','y','z') NOT NULL,
    crest_factor   DOUBLE,
    vdv_contrib    DOUBLE,                 -- this event's share of the VDV
    raw_peak_ms2   DOUBLE,                 -- unweighted, to spot clipping
    clipped        TINYINT(1)  NOT NULL DEFAULT 0,

    -- position at the trigger sample
    lat            DOUBLE,
    lon            DOUBLE,
    speed_mps      DOUBLE,
    course_deg     DOUBLE,
    hdop           DOUBLE,
    satellites     TINYINT,
    pos_source     ENUM('interpolated','nearest','stale','none')
                   NOT NULL DEFAULT 'none',
    pos_error_s    DOUBLE,                 -- gap to nearest real fix

    waveform_path   VARCHAR(255),
    waveform_sha256 CHAR(64),

    created_at     DATETIME(3) NOT NULL DEFAULT CURRENT_TIMESTAMP(3),

    UNIQUE KEY shock_seq_uq (session_id, seq),
    KEY shock_time_idx (event_time),
    KEY shock_peak_idx (peak_ms2),
    KEY shock_latlon_idx (lat, lon),
    CONSTRAINT shock_session_fk FOREIGN KEY (session_id)
        REFERENCES session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ---------------------------------------------------------------------
-- Periodic exposure summaries. These, not the individual events, are
-- what answer "is this operator over the action value".
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS exposure_interval (
    interval_id    BIGINT AUTO_INCREMENT PRIMARY KEY,
    session_id     BIGINT      NOT NULL,
    period_start   DATETIME(6) NOT NULL,
    period_end     DATETIME(6) NOT NULL,

    aw_x DOUBLE, aw_y DOUBLE, aw_z DOUBLE,
    vdv_x DOUBLE, vdv_y DOUBLE, vdv_z DOUBLE,
    peak_x DOUBLE, peak_y DOUBLE, peak_z DOUBLE,
    dominant_axis  ENUM('x','y','z'),
    crest_factor   DOUBLE,

    moving_s       DOUBLE      NOT NULL,
    idle_s         DOUBLE      NOT NULL,
    shock_count    INT         NOT NULL DEFAULT 0,

    -- normalised to an 8 h day from the assumed daily exposure
    exposure_hours DOUBLE,
    a8_ms2         DOUBLE,
    vdv8           DOUBLE,

    UNIQUE KEY exposure_uq (session_id, period_start),
    KEY exposure_session_idx (session_id, period_start),
    CONSTRAINT exposure_session_fk FOREIGN KEY (session_id)
        REFERENCES session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ---------------------------------------------------------------------
-- Views
-- ---------------------------------------------------------------------

-- Rough-spot clustering on a roughly 5 m grid. 0.000045 deg of latitude
-- is about 5 m; the longitude step is divided by cos(lat) so the cells
-- stay approximately square.
CREATE OR REPLACE VIEW hotspot_grid AS
SELECT
    ROUND(lat / 0.000045) * 0.000045                        AS cell_lat,
    ROUND(lon / (0.000045 / COS(RADIANS(lat))))
        * (0.000045 / COS(RADIANS(lat)))                    AS cell_lon,
    COUNT(*)                                                AS events,
    MAX(peak_ms2)                                           AS worst_peak,
    AVG(peak_ms2)                                           AS mean_peak,
    MIN(event_time)                                         AS first_seen,
    MAX(event_time)                                         AS last_seen
FROM shock_event
WHERE lat IS NOT NULL
  AND pos_source IN ('interpolated','nearest')
GROUP BY cell_lat, cell_lon
HAVING COUNT(*) >= 3;

-- Per-session headline numbers.
CREATE OR REPLACE VIEW session_summary AS
SELECT
    s.session_id,
    s.session_uuid,
    t.asset_tag,
    s.started_at,
    s.ended_at,
    COUNT(DISTINCT e.event_id)                     AS shock_count,
    MAX(e.peak_ms2)                                AS worst_peak_ms2,
    SUM(e.clipped)                                 AS clipped_events,
    (SELECT MAX(a8_ms2)  FROM exposure_interval x
       WHERE x.session_id = s.session_id)          AS max_a8_ms2,
    (SELECT MAX(vdv8)    FROM exposure_interval x
       WHERE x.session_id = s.session_id)          AS max_vdv8
FROM session s
LEFT JOIN truck t       ON t.truck_id = s.truck_id
LEFT JOIN shock_event e ON e.session_id = s.session_id
GROUP BY s.session_id, s.session_uuid, t.asset_tag, s.started_at, s.ended_at;

-- ---------------------------------------------------------------------
-- Running this on the Pi - three things that will bite
-- ---------------------------------------------------------------------
-- 1. SD cards die from write cycles. gps_fix at 5 Hz is the bulk of the
--    volume. Either drop the logging rate to 1 Hz, prune old fixes, or
--    move the MySQL datadir onto a USB SSD:
--       sudo systemctl stop mariadb
--       sudo rsync -av /var/lib/mysql/ /mnt/ssd/mysql/
--       # set datadir=/mnt/ssd/mysql in /etc/mysql/mariadb.conf.d/50-server.cnf
--
-- 2. A forklift cuts power with the key. InnoDB survives that only if it
--    actually flushed - keep the default and do NOT be tempted to relax:
--       innodb_flush_log_at_trx_commit = 1
--    Better still, fit a supercap or UPS HAT and shut down cleanly.
--
-- 3. Prune the track, keep the events. Something like:
--    DELETE FROM gps_fix
--     WHERE fix_time < NOW() - INTERVAL 30 DAY;
--    Events are small and worth keeping indefinitely.