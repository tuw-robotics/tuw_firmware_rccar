FROM espressif/idf:release-v5.4

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Vienna \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Update and install packages, configure locale and timezone, and update bashrc
RUN apt-get update -q && \
    apt-get install -yq sudo gosu lsb-release udev python3-pip locales git clang-format && \
    rm -rf /var/lib/apt/lists/* && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    echo "export IDF_EXPORT_QUIET=1" >> /root/.bashrc && \
    echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> /root/.bashrc

# Set the working directory
WORKDIR /workspaces/firmware

# Copy requirements and install Python packages
COPY requirements.txt .
RUN /bin/bash -c "source /opt/esp/idf/export.sh && pip3 install -r requirements.txt"

CMD ["/bin/bash"]
