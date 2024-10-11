FROM ubuntu:latest
WORKDIR /app
COPY . .

RUN apt-get update && \
    apt-get install -y \
    g++ \
    cmake \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libvtk9-dev \
    libopenni2-dev \
    libpcap-dev \
    libproj-dev \
    libpng-dev \
    libglew-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libtiff5-dev \
    libjpeg-dev \
    liblz4-dev \
    libzstd-dev \
    libxt-dev

RUN apt-get install -y libpcl-dev
RUN apt-get install -y libcgal-dev libcgal-qt5-dev

RUN apt-get clean && rm -rf /var/lib/apt/lists/*