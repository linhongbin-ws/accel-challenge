docker build -t accel:default .
docker run -t -i --gpu=all accel:default /bin/bash

