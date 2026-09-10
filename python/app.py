import time
import os
import MySQLdb  # standard mariadb/mysql client library
from flask import Flask, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app) # Allows the HTML page to fetch data without security errors

# Database connection configuration
DB_HOST = "mysql-server"  # Docker container name handles the internal routing
DB_USER = "root"
DB_PASSWORD = "B@zinga1"
DB_NAME = "VibrationDB"

def init_db():
    """Ensure our database table exists."""
    while True:
        try:
            db = MySQLdb.connect(host=DB_HOST, user=DB_USER, passwd=DB_PASSWORD, db=DB_NAME)
            cursor = db.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS sensor_data (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    sensor_value TEXT NOT NULL
                )
            """)
            db.commit()
            db.close()
            print("Database initialized successfully!")
            break
        except Exception as e:
            print(f"Waiting for database... Error: {e}")
            time.sleep(3)

@app.route('/api/data', methods=['GET'])
def get_data():
    """API endpoint for the HTML page to query."""
    try:
        db = MySQLdb.connect(host=DB_HOST, user=DB_USER, passwd=DB_PASSWORD, db=DB_NAME)
        cursor = db.cursor()
        cursor.execute("SELECT timestamp, sensor_value FROM sensor_data ORDER BY id DESC LIMIT 10")
        rows = cursor.fetchall()
        db.close()
        
        data_list = [{"timestamp": str(row[0]), "value": row[1]} for row in rows]
        return jsonify(data_list)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# -- HARDWARE READING LOGIC --
# In a real environment, you would use 'import serial' or 'import RPi.GPIO'
# Example: 
# ser = serial.Serial('/dev/ttyAMA0', 9600)
# data = ser.readline().decode('utf-8')

if __name__ == '__main__':
    init_db()
    # Run the API server on port 5000
    app.run(host='0.0.0.0', port=5000)