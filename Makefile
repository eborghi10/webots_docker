.PHONY: webots_docker
webots_docker:
	docker build --build-arg uid=$(shell id -u) -t webots_docker .
