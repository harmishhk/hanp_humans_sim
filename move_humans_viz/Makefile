all: build

.PHONY: prepare build run stop

prepare:
	@docker run --rm -it -v $(CURDIR):/workspace harmish/typescript typings install --save

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
