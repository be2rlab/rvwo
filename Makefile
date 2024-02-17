SHELL := /bin/bash

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
ROOT_DIR := $(MKFILE_DIR)

DATA_DIR ?= /home/sber20/dev/data

PARAMETERS := ROOT_DIR=$(ROOT_DIR) \
	DATA_DIR=$(DATA_DIR)

prepare-terminal-for-visualization:
	xhost local:docker

build-rvwo:
	cd $(ROOT_DIR)/docker && \
	$(PARAMETERS) \
	docker compose build rvwo

run-rvwo:
	cd $(ROOT_DIR)/docker && \
	$(PARAMETERS) \
	docker compose run rvwo

