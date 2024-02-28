FROM husarnet/ros2router:1.9.0

COPY ./filter.yaml /
RUN apt update && apt install vim -y

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "ddsrouter", "-c", "/var/tmp/DDS_ROUTER_CONFIGURATION.yaml" ]

