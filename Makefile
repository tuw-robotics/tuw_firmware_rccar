.PHONY: add-rccar_msgs

add-rccar_msgs:
	git clone --depth 1 https://github.com/lffelmann/rccar_msgs.git /workspaces/tuw_firmware_rccar/managed_components/micro_ros_espidf_component/extra_packages/rccar_msgs
	idf.py clean-microros