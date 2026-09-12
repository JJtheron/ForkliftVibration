import time
import os
import threading
import MySQLdb
from flask import Flask, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

DB_HOST = os.environ["DB_HOST"]
DB_PORT = int(os.environ.get("DB_PORT", "3306"))
DB_NAME = os.environ["DB_NAME"]
DB_USER = os.environ["DB_USER"]
DB_PASSWORD = os.environ["DB_PASSWORD"]


def init_db():
    """Initializes the database and updates the schema to store CPU temperature."""
    while True:
        try:
            db = MySQLdb.connect(host=DB_HOST, user=DB_USER, passwd=DB_PASSWORD, db=DB_NAME)
            cursor = db.cursor()
            # Added cpu_temp field to store decimal degree information
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS sensor_data (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    sensor_value TEXT NOT NULL,
                    cpu_temp DECIMAL(5,2) DEFAULT 0.00
                )
            """)
            db.commit()
            db.close()
            print("Database and schema initialized successfully!")
            break
        except Exception as e:
            print(f"Waiting for database... Error: {e}")
            time.sleep(3)

def get_pi_temperature():
    """Reads the Raspberry Pi CPU temperature directly from the system kernel."""
    try:
        if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                # Value is returned in millidegrees (e.g. 45123 = 45.1°C)
                return round(float(f.read().strip()) / 1000.0, 2)
        return 0.0  # Fallback if file isn't present in testing environments
    except Exception as e:
        print(f"Error reading CPU temperature: {e}")
        return 0.0

def telemetry_logger():
    """Background loop that captures hardware sensor/serial streams and CPU temperatures."""
    init_db()
    while True:
        try:
            # 1. Capture your existing IO/Serial strings here
            dummy_sensor_string = "Forklift Node Active" 
            
            # 2. Capture live Pi core temperature
            current_temp = get_pi_temperature()
            
            # 3. Log everything to the MariaDB instance
            db = MySQLdb.connect(host=DB_HOST, user=DB_USER, passwd=DB_PASSWORD, db=DB_NAME)
            cursor = db.cursor()
            cursor.execute(
                "INSERT INTO sensor_data (sensor_value, cpu_temp) VALUES (%s, %s)",
                (dummy_sensor_string, current_temp)
            )
            db.commit()
            db.close()
            
            print(f"Logged Telemetry - Temp: {current_temp}°C")
        except Exception as e:
            print(f"Telemetry logging error: {e}")
            
        time.sleep(10) # Log a data point every 10 seconds

@app.route('/api/data', methods=['GET'])
def get_data():
    """Queries the database to return the latest logs to the HTML webpage."""
    try:
        db = MySQLdb.connect(host=DB_HOST, user=DB_USER, passwd=DB_PASSWORD, db=DB_NAME)
        cursor = db.cursor()
        cursor.execute("SELECT timestamp, sensor_value, cpu_temp FROM sensor_data ORDER BY id DESC LIMIT 10")
        rows = cursor.fetchall()
        db.close()
        
        data_list = [{"timestamp": str(row[0]), "value": row[1], "cpu_temp": float(row[2])} for row in rows]
        return jsonify(data_list)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    # Start the hardware logger thread independently from the web API server
    log_thread = threading.Thread(target=telemetry_logger, daemon=True)
    log_thread.start()
    
    app.run(host='0.0.0.0', port=5000)
