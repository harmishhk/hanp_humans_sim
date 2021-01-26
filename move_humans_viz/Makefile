all: stop prepare build run

.PHONY: prepare build build-image run stop
SPECTRE_URL =  https://github.com/picturepan2/spectre/raw/232ca5e1b54f428b5ec5f06b91f0d084a0c76013/dist/spectre.min.css

prepare:
	@docker run --rm -it -v $(CURDIR):/workspace harmish/typescript typings install --save
	@# @docker run --rm -it -v $(HOME)/ros:$(HOME)/ros -w $(PWD) harmish/typescript typings install --save
	@docker run --rm -it -v $(CURDIR):/workspace busybox test -e /workspace/dist/spectre.min.css || wget $(SPECTRE_URL) -P dist
	@# @cp $(HOME)/ros/web/rosjs-typings/rosmsgs/build/rosmsgs.js $(CURDIR)/dist/
	@# @cp $(HOME)/ros/web/ros3dnavts/build/ros3dnav.js $(CURDIR)/dist/

build:
	@docker run --rm -v $(CURDIR):/workspace harmish/typescript tsc -t ES6 -p /workspace || true

build-image:
	@docker build -t harmish/move_humans_viz .

run: build-image
	@docker run -d --name=move_humans_viz \
     --restart=unless-stopped \
     -p 80:80 \
     harmish/move_humans_viz

stop::
	@docker stop move_humans_viz >/dev/null 2>&1 || true
	@docker rm move_humans_viz >/dev/null 2>&1 || true
