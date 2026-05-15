# Ungula Motor Library
# Copyright © 2026, Alex Conesa
# MIT License

cmake_minimum_required(VERSION 3.15)

if(ESP_PLATFORM)
	# Build UngulaMotor as an ESP-IDF component
	idf_component_register(INCLUDE_DIRS src)
	return()
endif()

project(UngulaMotor VERSION 3.0.0)

# Host-side standalone build placeholder. Host tests live under tests/
# and glob src/ themselves via their own CMakeLists.txt — nothing to
# add_subdirectory here today.
