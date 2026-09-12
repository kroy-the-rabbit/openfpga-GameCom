PODMAN ?= podman
IMAGE ?= localhost/pocket-quartus:25.1std
SIMIMAGE ?= localhost/gamecom-verify:1
STORAGEIMAGE ?= $(SIMIMAGE)
STORAGE_JOBS ?= 1
ifeq ($(notdir $(PODMAN)),docker)
RUNAS := --user $(shell id -u):$(shell id -g)
else ifeq ($(shell id -u),0)
RUNAS := --security-opt label=disable
else
RUNAS := --userns=keep-id --security-opt label=disable
endif
ROMSET_MOUNT = $(if $(ROMSET),-v "$(abspath $(ROMSET)):/romset.zip:ro" -e ROMSET=/romset.zip,)
SIMRUN = $(PODMAN) run --rm $(RUNAS) \
	-v "$(CURDIR):/work" -w /work -e HOME=/tmp $(ROMSET_MOUNT) $(SIMIMAGE)

.PHONY: gamecom test test-unit test-storage test-boot report sim-image
gamecom:
	PODMAN=$(PODMAN) IMAGE=$(IMAGE) QUARTUS_BIN="$(QUARTUS_BIN)" SEED=$(SEED) SKIP_COMPILE=$(SKIP_COMPILE) \
	FITTER_EFFORT="$(FITTER_EFFORT)" NPROC="$(NPROC)" RELEASE_NAME=$(RELEASE_NAME) tools/podman/build.sh
test: test-unit test-storage
test-unit:
	$(SIMRUN) python3 tools/sim/run_all.py
test-storage:
	$(PODMAN) run --rm $(RUNAS) \
	-v "$(CURDIR):/work" -w /work -e HOME=/tmp $(ROMSET_MOUNT) \
	-e ROMSET_ALL=$(ROMSET_ALL) -e STORAGE_JOBS=$(STORAGE_JOBS) -e STORAGE_SIZES=$(STORAGE_SIZES) \
	$(if $(STORAGE_REPORT),-e STORAGE_REPORT="$(STORAGE_REPORT)",) $(STORAGEIMAGE) \
	python3 tools/sim/run_storage_integration.py
test-boot:
	$(PODMAN) run --rm $(RUNAS) \
	-v "$(CURDIR):/work" -w /work -e HOME=/tmp $(ROMSET_MOUNT) $(STORAGEIMAGE) \
	python3 tools/sim/run_rom_boot.py $(ARGS)
report:
	tools/podman/report.sh
sim-image:
	$(PODMAN) build --security-opt label=disable -t $(SIMIMAGE) -f tools/podman/Containerfile.verify tools/podman
