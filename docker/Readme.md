# Description:

Files in this folder is used to make docker image of intel/kinetic:moveit_handson base environment.

Dockerfile -- the base file which will be automatically loaded when run docker build command.

setup_docker_display.sh -- set the container to display a pop-up x window.
install_docker.sh -- install docker-ce on ubuntu16.04 and newer.

# Build command:
First, clone the repo to your local system.
```
git clone https://github.com/RoboticsYY/moveit_handson.git
cd moveit_handson/docker/
sudo docker build -t intel/kinetic:moveit_handson .
```
If proxies are needed:
```
sudo docker build -t intel/kinetic:moveit_handson --build-arg http_proxy=http://proxy.my.com:prot_number --build-arg https_proxy=http://proxy.my.com:prot_number .
```
# Run docker image
After the project runs, there will be a pop-up x window, you need to set the operating environment first.
```
./moveit_handson/setup_docker_display.sh
```
Run this docker images
```
sudo docker run -t -i --rm -v $XSOCK:$XSOCK:rw -v $XAUTH:$XAUTH:rw -e XAUTHORITY=${XAUTH} -e DISPLAY --name moveit_handson intel/kinetic:moveit_handson bash
```

Note: If you haven't already installed or want more information on how to use docker, please see the article here for more information:
https://docs.docker.com/install/
