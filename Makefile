# ----------------------------------------------------------------------
#  ROS-Humble Docker Development
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file

# ---------HUMBLE----------
# No GPU
humble.build:
	@./docker/scripts/build.bash --ros-distro=humble

# CUDA 11.8 x86_64
humble.build.cuda:
	@./docker/scripts/build.bash --ros-distro=humble --use-cuda --cuda-image=$(cuda-image) --cuda-version=$(cuda-version)

# Jetson L4T 35.2.1
humble.build.jetson.35.2.1:
	@./docker/scripts/build.bash --ros-distro=humble --l4t=35.2.1 

te3003.build:
	@./docker/scripts/build.bash --ros-distro=te3003

te3003.build.cuda:
	@./docker/scripts/build.bash --ros-distro=te3003 --use-cuda --cuda-image=$(cuda-image) --cuda-version=$(cuda-version)

# ----------------------------CREATE------------------------------------

# Create containers, receive arguments --volume
humble.create:
	@./docker/scripts/run.bash --ros-distro=humble --volumes=$(volumes) --name=$(name)

# Create containers with CUDA support
humble.create.cuda:
	@./docker/scripts/run.bash --ros-distro=humble --use-cuda --volumes=$(volumes) --name=$(name)

humble.create.jetson.35.2.1:
	@./docker/scripts/run.bash --ros-distro=humble --l4t=35.2.1 --volumes=$(volumes) --name=$(name)

te3003.create:
	@./docker/scripts/run.bash --ros-distro=te3003 --volumes=$(volumes) --name=$(name)

te3003.create.cuda:
	@./docker/scripts/run.bash --ros-distro=te3003 --use-cuda --volumes=$(volumes) --name=$(name)

# ----------------------------START------------------------------------
# Start containers
humble.up:
	@ if [ -n "$(DISPLAY)" ]; then xhost +; fi
	@docker start ros-humble 

te3003.up:
	@ if [ -n "$(DISPLAY)" ]; then xhost +; fi
	@docker start ros-te3003

# ----------------------------STOP------------------------------------
# Stop containers
humble.down:
	@docker stop ros-humble 

te3003.down:
	@docker stop ros-te3003

# ----------------------------RESTART------------------------------------
# Restart containers
humble.restart:
	@docker restart ros-humble 

te3003.restart:
	@docker restart ros-te3003

# ----------------------------LOGS------------------------------------
# Logs of the container
humble.logs:
	@docker logs --tail 50 ros-humble 

te3003.logs:
	@docker logs --tail 50 ros-te3003

# ----------------------------SHELL------------------------------------
# Fires up a bash session inside the container
humble.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) ros-humble bash

te3003.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) ros-te3003 bash

# ----------------------------REMOVE------------------------------------
# Remove container
humble.remove:
	@docker container rm ros-humble 

te3003.remove:
	@docker container rm ros-te3003

# ----------------------------------------------------------------------
#  General Docker Utilities

#: Show a list of images.
list-images:
	@docker image ls

#: Show a list of containers.
list-containers:
	@docker container ls -a