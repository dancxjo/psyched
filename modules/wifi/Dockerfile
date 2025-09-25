FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    hostapd dnsmasq avahi-daemon iproute2 python3 && \
    rm -rf /var/lib/apt/lists/*
COPY hostapd.conf /etc/hostapd/hostapd.conf
COPY dnsmasq.conf /etc/dnsmasq.conf
COPY avahi-daemon.conf /etc/avahi/avahi-daemon.conf
COPY entrypoint.sh /entrypoint.sh
COPY www/ /srv/www/
RUN chmod +x /entrypoint.sh
CMD ["/entrypoint.sh"]
