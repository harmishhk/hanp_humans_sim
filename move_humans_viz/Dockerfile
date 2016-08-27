FROM httpd:2.4-alpine

COPY ./build/moveHumansViz.js /usr/local/apache2/htdocs/
COPY ./index.html /usr/local/apache2/htdocs/

RUN chmod -R a+rx /usr/local/apache2/htdocs

EXPOSE 80
