# Umbreon Roborace — Makefile
# Requires: Python 3.8+, pip

PYTHON   ?= python
PIP      ?= pip
CLI      ?= "$(LOCALAPPDATA)/Programs/arduino-ide/resources/app/lib/backend/resources/arduino-cli"
SIM_DIR   = simulation
DASH_DIR  = dashboard
PORT      ?= 8080
SIM_PORT  ?= 8023
FQBN     ?= rp2040:rp2040:rpipico2
SERIAL   ?= auto

# ─── Setup ────────────────────────────────────────────────────────────────
.PHONY: install
install:  ## Install Python dependencies
	$(PIP) install -r $(DASH_DIR)/requirements.txt

# ─── Dashboard ────────────────────────────────────────────────────────────
.PHONY: dashboard web
dashboard: web
web:  ## Start web dashboard on localhost:8080
	cd $(DASH_DIR) && $(PYTHON) server.py $(PORT)

.PHONY: dashboard-tk
dashboard-tk:  ## Start tkinter desktop dashboard
	cd $(DASH_DIR) && $(PYTHON) app.py

# ─── Simulator ────────────────────────────────────────────────────────────
.PHONY: sim
sim:  ## Run live simulation (matplotlib animation)
	cd $(SIM_DIR) && $(PYTHON) sim.py

.PHONY: sim-fast
sim-fast:  ## Run pre-computed simulation and plot results
	cd $(SIM_DIR) && $(PYTHON) sim.py --fast

.PHONY: sim-bridge bridge
sim-bridge: bridge
bridge:  ## Run simulation with TCP bridge for dashboard testing
	cd $(SIM_DIR) && $(PYTHON) sim.py --bridge

# ─── Combined (sim + dashboard) ──────────────────────────────────────────
.PHONY: demo
demo:  ## Start sim bridge + web dashboard (two background processes)
	@echo "Starting simulator bridge on port $(SIM_PORT)..."
	@cd $(SIM_DIR) && $(PYTHON) sim.py --bridge &
	@sleep 1
	@echo "Starting web dashboard on http://localhost:$(PORT)"
	@echo "Connect to localhost:$(SIM_PORT) in the dashboard"
	@cd $(DASH_DIR) && $(PYTHON) server.py $(PORT)

# ─── Firmware ─────────────────────────────────────────────────────────
.PHONY: compile
compile:  ## Compile firmware (arduino-cli)
	$(CLI) compile -b $(FQBN) .

.PHONY: upload
upload:  ## Compile & upload firmware (set SERIAL=COM4 or /dev/ttyACM0)
ifeq ($(SERIAL),auto)
	$(CLI) compile -b $(FQBN) . --upload
else
	$(CLI) compile -b $(FQBN) . --upload -p $(SERIAL)
endif

# ─── ROS2 (Docker) ───────────────────────────────────────────────────────
.PHONY: ros2-build
ros2-build:  ## Build ROS2 Docker image
	cd ros2 && docker compose build

.PHONY: ros2-run
ros2-run:  ## Run ROS2 bridge (connects to car at 192.168.4.1:23)
	cd ros2 && docker compose up umbreon-bridge

.PHONY: ros2-sim
ros2-sim:  ## Run ROS2 bridge against simulator (localhost:8023)
	cd ros2 && docker compose --profile sim up umbreon-bridge-sim

.PHONY: ros2-shell
ros2-shell:  ## Open a shell in the ROS2 container
	cd ros2 && docker compose run --rm umbreon-bridge bash

# ─── Helpers ──────────────────────────────────────────────────────────────
.PHONY: clean
clean:  ## Remove Python caches
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true

.PHONY: help
help:  ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
