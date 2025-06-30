set(SOURCES
    # playground/misc/servo_calibration.c
    main.c
    can/can_driver.c
    can/can_dispatcher.c
    can/can_rtr.c
    can/can_interface.c
    odrive/odrive.c
    mros/mros.c
    mros/mros_transport.c
    mros/mros_param.c
    kinematics/kinematics.c
    utils/indicator_led.c
    utils/jtag_debug.c
    utils/math_utils.c
    servo/servo.c
    robot_controller/robot_controller.c
)

idf_component_register(SRCS ${SOURCES}
    INCLUDE_DIRS "."
)