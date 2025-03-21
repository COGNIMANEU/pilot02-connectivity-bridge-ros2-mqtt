FROM ros:humble

RUN apt update && apt install -y python3-pip python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*

COPY ./ /test

