FROM python:3.11-alpine

# Install build dependencies required for compiling low-level C libraries on ARM
RUN apk add --no-cache gcc musl-dev mariadb-connector-c-dev linux-headers

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["python", "app.py"]