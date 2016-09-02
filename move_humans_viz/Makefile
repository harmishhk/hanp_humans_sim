all: prepare build

.PHONY: prepare build run stop
SPECTRE_URL =  https://github.com/picturepan2/spectre/raw/232ca5e1b54f428b5ec5f06b91f0d084a0c76013/dist/spectre.min.css

prepare:
	@docker run --rm -it -v $(CURDIR):/workspace harmish/typescript typings install --save
	@docker run --rm -it -v $(CURDIR):/workspace busybox test -e /workspace/dist/spectre.min.css || wget $(SPECTRE_URL) -P dist

build:
	@docker run --rm -v $(CURDIR):/workspace harmish/typescript tsc -p /workspace
	@docker build -t harmish/move_humans_viz .

run:
	@docker run -d --name=move_humans_viz \
     --restart=unless-stopped \
     -p 80:80 \
     harmish/move_humans_viz

stop::
	@docker stop move_humans_viz >/dev/null 2>&1 || true
	@docker rm move_humans_viz >/dev/null 2>&1 || true
