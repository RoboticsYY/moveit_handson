# Description:

Files in this folder is used to make docker image of intel/kinetic:moveit_handson base environment.

Dockerfile -- the base file which will be automatically loaded when run docker build command

setup_docker_display.sh -- set the container to display a pop-up

# Build command:
First, clone the repo to your local system.
```
sudo docker build -t intel/kinetic:moveit_handson .
```
If proxies are needed:
```
sudo docker build -t intel/kinetic:moveit_handson --build-arg http_proxy=http://proxy.my.com:### --build-arg https_proxy=http://proxy.my.com:### .
```
# Run docker image
After the project runs, there will be a pop-up window, you need to set the operating environment first.
```
./setup_docker_display.sh
```
Run this docker images
```

sudo docker run -t -i --rm -v $XSOCK:$XSOCK:rw -v $XAUTH:$XAUTH:rw -e XAUTHORITY=${XAUTH} -e DISPLAY --name moveit_handson intel/kinetic:moveit_handson bash
or
sudo docker run -t -i --rm -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw -e XAUTHORITY=/tmp/.docker.xauth -e DISPLAY --name moveit_handson intel/kinetic:moveit_handson bash
```

Run this command if you want to open multiple container terminals
```
sudo docker exec -t -i moveit_handson bash
```

If you want to run multiple containers, please modify "--name"

Note: If you haven't already installed or want more information on how to use docker, please see the article here for more information:
https://docs.docker.com/install/
