# kinekit — common shortcuts wrapping docker/build.sh and Pi deployment.
#
# Override defaults from the command line, e.g.:
#   make build PLATFORM=linux/amd64
#   make deploy PI=pi@otherbox.local

# --- Configuration ---------------------------------------------------------

PI         ?= pi@zero.local
PI_DIR     ?= /home/pi/kinekit
PLATFORM   ?= linux/arm64
OUT        := out/$(subst /,-,$(PLATFORM))
ARTIFACTS  := kinegram kinemetry libkinecore.so
KEY        ?= $(HOME)/.ssh/id_rsa

.DEFAULT_GOAL := help

# --- Help -----------------------------------------------------------------

.PHONY: help
help: ## Show this help
	@awk 'BEGIN {FS = ":.*##"; printf "Usage: make \033[36m<target>\033[0m\n\nTargets:\n"} \
	      /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo
	@echo "Variables (override on the command line):"
	@echo "  PI        = $(PI)"
	@echo "  PI_DIR    = $(PI_DIR)"
	@echo "  PLATFORM  = $(PLATFORM)"
	@echo "  KEY       = $(KEY)"

# --- Build ----------------------------------------------------------------

.PHONY: build
build: ## Cross-build for PLATFORM (default: linux/arm64 — Pi Zero 2 W)
	PLATFORM=$(PLATFORM) ./docker/build.sh

.PHONY: clean
clean: ## Remove build artefacts
	rm -rf out

# --- Deploy / run / test --------------------------------------------------

.PHONY: deploy
deploy: build ## Build then scp artefacts to PI:PI_DIR
	ssh $(PI) "mkdir -p $(PI_DIR)"
	scp $(addprefix $(OUT)/,$(ARTIFACTS)) $(PI):$(PI_DIR)/
	@echo "==> Deployed to $(PI):$(PI_DIR)"

.PHONY: run
run: ## Run kinegram on PI (must be deployed first)
	ssh $(PI) "LD_LIBRARY_PATH=$(PI_DIR) $(PI_DIR)/kinegram"

.PHONY: test
test: deploy ## Deploy and verify kinegram prints its version on PI
	@out=$$(ssh $(PI) "LD_LIBRARY_PATH=$(PI_DIR) $(PI_DIR)/kinegram"); \
	echo "$$out"; \
	echo "$$out" | grep -q "kinecore 0.1.0" \
	    && echo "==> test PASS" \
	    || (echo "==> test FAIL"; exit 1)

.PHONY: ssh
ssh: ## SSH into PI
	ssh $(PI)

# --- SSH key setup --------------------------------------------------------

.PHONY: ssh-key
ssh-key: ## Copy local public key to PI — will prompt for the Pi password once
	ssh-copy-id -i $(KEY).pub $(PI)
	@echo "==> Key installed. Try: make ssh"
