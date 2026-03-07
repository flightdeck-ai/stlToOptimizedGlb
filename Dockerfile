FROM ubuntu:24.10

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libboost-system-dev \
    libboost-filesystem-dev \
    libassimp-dev \
    git

WORKDIR /app

COPY CMakeLists.txt .
COPY main.cpp .

RUN mkdir build && \
    cd build && \
    cmake .. && \
    make

CMD ["./build/stlToOptimizedGlb"]