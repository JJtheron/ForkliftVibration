FROM python:3.11-slim-bookworm

# Keep the image compatible with Raspberry Pi OS ARM64 wheels. SciPy and NumPy
# are substantially easier to install on Debian/glibc than Alpine/musl.
RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
		gcc g++ linux-libc-dev libmariadb-dev libopenblas-dev \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["python", "app.py"]
