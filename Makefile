# Detect architecture
ARCH := $(shell uname -m)
ifeq ($(ARCH),x86_64)
	PLATFORM := x86
	DOCKER_IMAGE := build_camd_x86
else
	PLATFORM := arm
	DOCKER_IMAGE := build_rpi_capturer_arm
endif

# Build only the image for detected architecture
docker:
	@echo "Building Docker image for $(PLATFORM) architecture"
	docker build -t $(DOCKER_IMAGE) -f Dockerfile.$(PLATFORM) .

# Run docker with auto-detected platform image
docker-run: docker
	@echo "Running Docker for $(PLATFORM) architecture"
	docker run -it -v $(shell pwd)/:/home -w /home $(DOCKER_IMAGE) /bin/bash

build-libcamera: docker
	docker run -it -v $(shell pwd)/:/home -w /home $(DOCKER_IMAGE) bash -c 'meson setup --wipe build && ninja -C build'

# Build with auto-detected platform image
build-capturer: docker
	@echo "Building with $(PLATFORM) architecture"
	docker run --rm -v $(shell pwd):/home -w /home $(DOCKER_IMAGE) bash -c 'cd rpi_capturer/ && rm -r build && mkdir build && cd build && cmake .. && make'

# Clean with auto-detected platform image
clean:
	docker run --rm -v $(shell pwd):/home -w /home $(DOCKER_IMAGE) bash -c 'cd capturer && make clean'

